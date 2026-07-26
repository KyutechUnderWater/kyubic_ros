"""ミッションの状態遷移(FSM)。blueRovControl/flow.py からの移植。

低レベルの推力計算は ctx.tick_fn(dx, dy, dz)(node.py が ControlLoop.tick を
束縛したcallback)に完全に委譲し、このファイルは「今どのStateで、次にどの
Stateへ行くか」の判断だけを行う(旧実装と同じ設計方針)。

旧実装との主な違い:
  - 自前QEKF(ctx.position/ctx.quaternion を control_tick が毎回書き戻す設計)から
    /localization/odom の購読値(node.pyがmission_tick呼び出し前に
    ctx.position/ctx.orientation_deg へ書き込む)に変わった。
  - 音響(SBL)/画像処理の実データは bluerov_control_msgs・planner_msgs トピックの
    購読値(ctx.pinger_direction / ctx.wrench_plan / ctx.buoy_detection、いずれも
    未受信ならNone)を使う。未受信時に例外を投げていた旧実装
    (perception.get_latest_*がNotImplementedError)と異なり、安全側
    (その場で待機/検出なし扱い)にフォールバックする。
    SEARCH_HYDROPHONEの移動目標(dx,dy,dz)は、このパッケージ内では計算しない。
    音響班のsbl_controller_nodeが方角(ctx.pinger_direction)から決定し
    planner_msgs/WrenchPlanとして発行した絶対座標(ctx.wrench_plan.targets)を
    そのまま追従する(§詳細はbluerov_control/README.md参照)。
  - 深度/オフセット定数(7.0m, 2.5m, 0.5m, 4.0m)や各種閾値は ctx.params
    (node.pyがROS2パラメータから構築するMissionParams)経由で読む。
"""

from dataclasses import dataclass
from enum import Enum, auto

import numpy as np

from . import perception


class State(Enum):
    INIT = auto()
    DESCEND_TO_7M = auto()
    SEARCH_HYDROPHONE = auto()
    VISUAL_DIVE = auto()
    VISUAL_CHECK_MIC = auto()
    VISUAL_DETECT = auto()
    VISUAL_ATTACK = auto()
    VISUAL_POST_CHECK = auto()
    HOLD_TARGET = auto()
    HYDRO_DIVE = auto()
    HYDRO_APPROACH = auto()
    HYDRO_NEAR_SURFACE = auto()
    HYDRO_CHECK_MIC = auto()
    SURFACE = auto()
    RETURN_HOME = auto()
    DONE = auto()
    EMERGENCY = auto()


@dataclass
class MissionParams:
    """ctx.params として渡す、FSMが参照する閾値・定数一式(node.pyがROS2 paramから構築)。"""

    init_wait_sec: float
    search_timeout_sec: float
    mic_confirm_timeout_sec: float
    hydrophone_pitch_threshold_deg: float
    hydrophone_score_threshold: float
    yolo_confidence_threshold: float
    yolo_stale_timeout_sec: float
    descend_depth_m: float
    dive_offset_m: float
    post_check_rise_offset_m: float
    surface_rise_offset_m: float


@dataclass
class MissionContext:
    params: MissionParams
    tick_fn: object  # callable: (dx, dy, dz) -> control.ControlResult。node.pyが毎tick差し替える

    elapsed_time: float = 0.0
    now_sec: float = 0.0

    position: np.ndarray = None  # 直近の /localization/odom から得た現在位置(NED) [x, y, z_depth]
    orientation_deg: tuple = (
        None  # 直近の /localization/odom から得た現在姿勢 (roll, pitch, yaw) [deg]
    )

    target_position: np.ndarray = None  # 現在Stateが目指す絶対座標(NED)。State切替時にNoneへ戻す
    discovered_target: np.ndarray = (
        None  # SEARCH_HYDROPHONE/VISUAL_DETECTが発見した対象の絶対座標(NED)
    )
    attack_retry_started_at: float = (
        None  # マイク確認待ちループに入った時刻。discovered_target確定時に一度だけ設定
    )

    pinger_direction: object = (
        None  # 最新の bluerov_control_msgs/PingerDirection(未受信ならNone)
    )
    wrench_plan: object = (
        None  # sbl_controller_nodeが発行する最新の planner_msgs/WrenchPlan(未受信ならNone)
    )
    buoy_detection: object = None  # 最新の bluerov_control_msgs/BuoyDetection(未受信ならNone)
    image_processing_available: bool = True  # デバッグ用パラメータで上書き可能(README参照)
    mic_data_from_above: bool = False  # デバッグ用パラメータで上書き可能(README参照)

    _state_entered_at: float = 0.0  # 内部管理用。現Stateに入った時のelapsed_time
    _previous_state: State = None  # 内部管理用。前回tickのState(切り替わり検知用)


def _time_in_state(ctx: MissionContext) -> float:
    """現在のStateに入ってからの経過時間 [s]。タイムアウト判定はこれを使う
    (ctx.elapsed_time はミッション開始からの累積時間なので、そのまま使うと誤判定する)"""
    return ctx.elapsed_time - ctx._state_entered_at


def _orientation_or_zero(ctx: MissionContext) -> tuple:
    return ctx.orientation_deg if ctx.orientation_deg is not None else (0.0, 0.0, 0.0)


def _position_or_zero(ctx: MissionContext) -> np.ndarray:
    return ctx.position if ctx.position is not None else np.zeros(3)


_WRENCH_PLAN_Z_MODE_DEPTH = 0  # planner_msgs/WrenchPlan.Z_MODE_DEPTH。flow.pyはROSメッセージ型を
# 直接importしない方針(node.py側でのみ型を扱う)のため、数値として複製している。


def _wrench_plan_absolute_target(ctx: MissionContext) -> np.ndarray | None:
    """最新のWrenchPlan(sbl_controller_nodeが発行)から絶対目標座標(NED)を取り出す。
    未受信ならNone。

    targets.master/slaveやhas_master/has_slaveはwrench_planner側の多段(PDLA等)
    処理専用のフィールドで、SEARCH_HYDROPHONEが必要とする単一の絶対目標としては
    使わないため無視し、常にtargets.x/y/zのみを見る。
    z_modeがZ_MODE_ALTITUDEの場合、bluerov_controlのposition表現は常にNEDの
    z_depthであり単純に置き換えられないため、z成分は信用せず現在深度を維持する
    (安全側)。
    """
    plan = ctx.wrench_plan
    if plan is None:
        return None
    if plan.z_mode == _WRENCH_PLAN_Z_MODE_DEPTH:
        z = plan.targets.z
    else:
        z = _position_or_zero(ctx)[2]
    return np.array([plan.targets.x, plan.targets.y, z])


def _run_control_tick(ctx: MissionContext, dx, dy, dz):
    """ctx.tick_fn(dx, dy, dz) を呼ぶ共通処理。"""
    return ctx.tick_fn(dx, dy, dz)


def _move_toward(ctx: MissionContext, absolute_target_fn):
    """
    「固定の目標地点に向かう」Stateの共通処理。
    ctx.target_position が未設定なら absolute_target_fn(ctx) で一度だけ決定し、
    以降は毎tick (target_position - 現在位置) を計算して tick_fn を呼ぶ。
    """
    if ctx.target_position is None:
        ctx.target_position = absolute_target_fn(ctx)

    dx, dy, dz = ctx.target_position - _position_or_zero(ctx)
    return _run_control_tick(ctx, dx, dy, dz)


def _with_mic_confirm_timeout(handler):
    """
    discovered_target確定後、マイクで確認が取れるまでのリトライ系State
    (VISUAL_DIVE〜VISUAL_POST_CHECK, HOLD_TARGET〜HYDRO_CHECK_MIC)に共通で適用する
    タイムアウトガード。ctx.attack_retry_started_at から params.mic_confirm_timeout_sec
    を超えたら、実際のハンドラを呼ばずに強制的にEMERGENCYへ遷移する。
    """

    def wrapped(ctx: MissionContext) -> State:
        if (
            ctx.attack_retry_started_at is not None
            and ctx.elapsed_time - ctx.attack_retry_started_at > ctx.params.mic_confirm_timeout_sec
        ):
            return State.EMERGENCY
        return handler(ctx)

    return wrapped


def _make_relative_dive_handler(
    offset_param_name: str, sign: float, next_state: State, self_state: State
):
    """
    「今いる場所からoffsetだけ潜る/浮上する」Stateのハンドラを生成する。
    VISUAL_DIVE, HYDRO_DIVE, VISUAL_POST_CHECK, HYDRO_NEAR_SURFACE, SURFACE が該当。
    offsetの大きさは ctx.params.<offset_param_name> [m] から毎回読む(ROS2パラメータで調整可能)。
    """

    def target_fn(ctx: MissionContext) -> np.ndarray:
        offset_m = getattr(ctx.params, offset_param_name) * sign
        return _position_or_zero(ctx) + np.array([0.0, 0.0, offset_m])

    def handler(ctx: MissionContext) -> State:
        result = _move_toward(ctx, target_fn)
        if result.emergency:
            return State.EMERGENCY
        if result.reached:
            return next_state
        return self_state

    return handler


def _make_absolute_depth_dive_handler(next_state: State, self_state: State):
    """
    「x,yは現在地を維持したまま、深度[m](ctx.params.descend_depth_m)だけを絶対値で
    目標にする」Stateのハンドラを生成する。DESCEND_TO_7M専用。
    """

    def target_fn(ctx: MissionContext) -> np.ndarray:
        base = _position_or_zero(ctx)
        return np.array([base[0], base[1], ctx.params.descend_depth_m])

    def handler(ctx: MissionContext) -> State:
        result = _move_toward(ctx, target_fn)
        if result.emergency:
            return State.EMERGENCY
        if result.reached:
            return next_state
        return self_state

    return handler


# ---- Stateハンドラ ----


def handle_init(ctx: MissionContext) -> State:
    if _time_in_state(ctx) < ctx.params.init_wait_sec:
        return State.INIT
    return State.DESCEND_TO_7M


def handle_search_hydrophone(ctx: MissionContext) -> State:
    if _time_in_state(ctx) > ctx.params.search_timeout_sec:
        return State.SURFACE

    pinger = ctx.pinger_direction
    target = _wrench_plan_absolute_target(ctx)
    if pinger is None or target is None:
        # PingerDirection/WrenchPlanのどちらかをまだ一度も受信していない。
        # 安全側に倒し、その場で待機する(片方だけの受信では動かない=単一ガード)。
        result = _run_control_tick(ctx, 0.0, 0.0, 0.0)
        if result.emergency:
            return State.EMERGENCY
        return State.SEARCH_HYDROPHONE

    # TODO(要ハードウェア仕様確認): 不等号の向きは "対象がハイドロフォンより深い(HYDRO_DIVEで
    # さらに+2.5m潜る設計と整合)" と仮定して pitch_deg > THRESHOLD (見下ろす角度が大きい=近い) に
    # している。ただしPingerDirectionの「正=対象が下方向」の定義と実機の取り付け向きに
    # 完全に依存するため、実機で必ず符号を検証すること。逆であれば `<` に戻す。
    if (
        perception.is_pinger_direction_usable(pinger, ctx.params.hydrophone_score_threshold)
        and pinger.pitch_deg > ctx.params.hydrophone_pitch_threshold_deg
    ):
        # 閾値を超えた瞬間、WrenchPlanが示していた絶対座標を「発見した対象の絶対座標」として
        # 記録する(sbl_controller_nodeの出力をそのまま信頼する)。以降のHOLD_TARGET/
        # HYDRO_DIVE/HYDRO_APPROACHはこれを参照するので、ここで一度も呼ばれなければ
        # 後続Stateは目標を持てない。
        ctx.discovered_target = target
        ctx.attack_retry_started_at = ctx.elapsed_time  # マイク確認待ちループのタイムアウト起点
        if ctx.image_processing_available:
            return State.VISUAL_DIVE
        return State.HOLD_TARGET

    # 探索中は毎tick、sbl_controller_nodeが発行する最新のWrenchPlan絶対目標との差分を
    # 計算して追従する(target_positionキャッシュは使わない。targetそのものが
    # 外部ノードから毎tick更新され続けるため)。
    dx, dy, dz = target - _position_or_zero(ctx)
    result = _run_control_tick(ctx, dx, dy, dz)
    if result.emergency:
        return State.EMERGENCY
    return State.SEARCH_HYDROPHONE


def handle_visual_check_mic(ctx: MissionContext) -> State:
    if ctx.mic_data_from_above:
        return State.SURFACE
    return State.VISUAL_DETECT


def handle_visual_detect(ctx: MissionContext) -> State:
    detection = ctx.buoy_detection
    if perception.is_buoy_detection_usable(
        detection,
        ctx.now_sec,
        ctx.params.yolo_confidence_threshold,
        ctx.params.yolo_stale_timeout_sec,
    ):
        roll_deg, pitch_deg, yaw_deg = _orientation_or_zero(ctx)
        body_offset = (
            detection.relative_buoy_x_m,
            detection.relative_buoy_y_m,
            detection.relative_buoy_z_m,
        )
        ctx.discovered_target = perception.body_offset_to_ned(
            _position_or_zero(ctx), roll_deg, pitch_deg, yaw_deg, body_offset
        )
        return State.VISUAL_ATTACK
    return State.VISUAL_DETECT


def _visual_attack_target(ctx: MissionContext) -> np.ndarray:
    # VISUAL_DETECTが記録したdiscovered_targetをそのまま使う。
    # ここがNoneなのは設計上ありえない(handle_visual_detectが必ず先に設定する)ため、
    # 万一Noneなら現在位置をそのまま返して足止めする(前進しない=安全側に倒す)。
    if ctx.discovered_target is not None:
        return ctx.discovered_target
    return _position_or_zero(ctx)


def handle_visual_attack(ctx: MissionContext) -> State:
    result = _move_toward(ctx, _visual_attack_target)
    if result.emergency:
        return State.EMERGENCY
    if result.reached:
        return State.VISUAL_POST_CHECK
    return State.VISUAL_ATTACK


def handle_hold_target(ctx: MissionContext) -> State:
    # discovered_targetはSEARCH_HYDROPHONE側で既に確定済み(handle_search_hydrophone参照)。
    # このStateは「保持している」ことを示す通過点であり、追加の計算は不要。
    return State.HYDRO_DIVE


def _hydro_approach_target(ctx: MissionContext) -> np.ndarray:
    # SEARCH_HYDROPHONEが記録したdiscovered_targetをそのまま使う(VISUAL_ATTACKと同じ考え方)。
    if ctx.discovered_target is not None:
        return ctx.discovered_target
    return _position_or_zero(ctx)


def handle_hydro_approach(ctx: MissionContext) -> State:
    result = _move_toward(ctx, _hydro_approach_target)
    if result.emergency:
        return State.EMERGENCY
    if result.reached:
        return State.HYDRO_NEAR_SURFACE
    return State.HYDRO_APPROACH


def handle_hydro_check_mic(ctx: MissionContext) -> State:
    if ctx.mic_data_from_above:
        return State.SURFACE
    return State.HOLD_TARGET


def handle_return_home(ctx: MissionContext) -> State:
    return State.DONE  # 元canvas注記通り、できなければ何もせずDONEでよい


handle_visual_dive = _make_relative_dive_handler(
    "dive_offset_m", 1.0, State.VISUAL_CHECK_MIC, State.VISUAL_DIVE
)
handle_hydro_dive = _make_relative_dive_handler(
    "dive_offset_m", 1.0, State.HYDRO_APPROACH, State.HYDRO_DIVE
)
handle_visual_post_check = _make_relative_dive_handler(
    "post_check_rise_offset_m", -1.0, State.VISUAL_CHECK_MIC, State.VISUAL_POST_CHECK
)
handle_hydro_near_surface = _make_relative_dive_handler(
    "post_check_rise_offset_m", -1.0, State.HYDRO_CHECK_MIC, State.HYDRO_NEAR_SURFACE
)
handle_surface = _make_relative_dive_handler(
    "surface_rise_offset_m", -1.0, State.RETURN_HOME, State.SURFACE
)

STATE_HANDLERS = {
    State.INIT: handle_init,
    State.DESCEND_TO_7M: _make_absolute_depth_dive_handler(
        State.SEARCH_HYDROPHONE, State.DESCEND_TO_7M
    ),
    State.SEARCH_HYDROPHONE: handle_search_hydrophone,
    State.VISUAL_DIVE: _with_mic_confirm_timeout(handle_visual_dive),
    State.VISUAL_CHECK_MIC: _with_mic_confirm_timeout(handle_visual_check_mic),
    State.VISUAL_DETECT: _with_mic_confirm_timeout(handle_visual_detect),
    State.VISUAL_ATTACK: _with_mic_confirm_timeout(handle_visual_attack),
    State.VISUAL_POST_CHECK: _with_mic_confirm_timeout(handle_visual_post_check),
    State.HOLD_TARGET: _with_mic_confirm_timeout(handle_hold_target),
    State.HYDRO_DIVE: _with_mic_confirm_timeout(handle_hydro_dive),
    State.HYDRO_APPROACH: _with_mic_confirm_timeout(handle_hydro_approach),
    State.HYDRO_NEAR_SURFACE: _with_mic_confirm_timeout(handle_hydro_near_surface),
    State.HYDRO_CHECK_MIC: _with_mic_confirm_timeout(handle_hydro_check_mic),
    State.SURFACE: handle_surface,
    State.RETURN_HOME: handle_return_home,
}


def mission_tick(ctx: MissionContext, state: State) -> State:
    """現在のStateに対応するハンドラを1回呼び、次のStateを返す。
    DONE/EMERGENCYなどハンドラを持たない終端Stateはそのまま返す(呼び出し側でループを止める)。

    Stateが前回の呼び出しから変わっていた場合、_time_in_state()の基準時刻と
    target_positionをここで一括リセットする(個々のハンドラがバラバラに管理しない)。
    """
    if state != ctx._previous_state:
        ctx._state_entered_at = ctx.elapsed_time
        ctx.target_position = None
        ctx._previous_state = state

    handler = STATE_HANDLERS.get(state)
    if handler is None:
        return state
    return handler(ctx)
