"""
ミッションの状態遷移(FSM)。BlueROV-Flow-refactored.canvas の対応表(state_table.md)を
そのままコード化したもの。低レベルの推力計算はcontrolfunction.control_tick()に完全に委譲し、
このファイルは「今どのStateで、次にどのStateへ行くか」の判断だけを行う。

センサ/認識系(BingerSearch, YOLO検出, マイク判定, ハイドロフォンpitch)はまだスタブ。
Step4で本物に置き換える。
"""
from dataclasses import dataclass
from enum import Enum, auto

import numpy as np

import params
import perception
from controlfunction import control_tick
from params import HYDROPHONE_PITCH_THRESHOLD_DEG


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
class MissionContext:
    elapsed_time: float = 0.0
    position: np.ndarray = None        # 直近のcontrol_tick()が返した現在位置(NED)
    quaternion: np.ndarray = None      # 直近のcontrol_tick()が返した現在姿勢[qw,qx,qy,qz]
    target_position: np.ndarray = None  # 現在Stateが目指す絶対座標(NED)。State切替時にNoneへ戻す
    discovered_target: np.ndarray = None  # SEARCH_HYDROPHONE/VISUAL_DETECTが発見した対象の絶対座標(NED)
    attack_retry_started_at: float = None  # マイク確認待ちループに入った時刻。discovered_target確定時に一度だけ設定
    _state_entered_at: float = 0.0      # 内部管理用。現Stateに入った時のelapsed_time


def _time_in_state(ctx: MissionContext) -> float:
    """現在のStateに入ってからの経過時間 [s]。タイムアウト判定はこれを使う
    (ctx.elapsed_time はミッション開始からの累積時間なので、そのまま使うと誤判定する)"""
    return ctx.elapsed_time - ctx._state_entered_at


def _run_control_tick(ctx: MissionContext, dx, dy, dz):
    """control_tick()を呼び、返ってきたposition/quaternionをctxに書き戻す共通処理。"""
    result = control_tick(dx, dy, dz)
    ctx.position = result.position
    ctx.quaternion = result.quaternion
    return result


def _move_toward(ctx: MissionContext, absolute_target_fn):
    """
    「固定の目標地点に向かう」Stateの共通処理。
    ctx.target_position が未設定なら absolute_target_fn(ctx) で一度だけ決定し、
    以降は毎tick (target_position - 現在位置) を計算して control_tick を呼ぶ。
    """
    if ctx.target_position is None:
        ctx.target_position = absolute_target_fn(ctx)

    if ctx.position is not None:
        dx, dy, dz = ctx.target_position - ctx.position
    else:
        # まだ一度もcontrol_tickを呼んでおらず現在位置が不明な初回のみの仮対応
        dx, dy, dz = ctx.target_position

    return _run_control_tick(ctx, dx, dy, dz)


def _with_mic_confirm_timeout(handler):
    """
    discovered_target確定後、マイクで確認が取れるまでのリトライ系State
    (VISUAL_DIVE〜VISUAL_POST_CHECK, HOLD_TARGET〜HYDRO_CHECK_MIC)に共通で適用する
    タイムアウトガード。ctx.attack_retry_started_at からparams.MIC_CONFIRM_TIMEOUT_SEC
    を超えたら、実際のハンドラを呼ばずに強制的にEMERGENCYへ遷移する。
    """
    def wrapped(ctx: MissionContext) -> State:
        if ctx.attack_retry_started_at is not None:
            if ctx.elapsed_time - ctx.attack_retry_started_at > params.MIC_CONFIRM_TIMEOUT_SEC:
                return State.EMERGENCY
        return handler(ctx)
    return wrapped


def _make_relative_dive_handler(offset, next_state, self_state):
    """
    「今いる場所からoffsetだけ潜る/浮上する」Stateのハンドラを生成する。
    VISUAL_DIVE, HYDRO_DIVE, VISUAL_POST_CHECK, HYDRO_NEAR_SURFACE, SURFACE が該当。
    """
    def target_fn(ctx: MissionContext) -> np.ndarray:
        base = ctx.position if ctx.position is not None else np.zeros(3)
        return base + np.array(offset)

    def handler(ctx: MissionContext) -> State:
        result = _move_toward(ctx, target_fn)
        if result.emergency:
            return State.EMERGENCY
        if result.reached:
            return next_state
        return self_state

    return handler


def _make_absolute_depth_dive_handler(depth_m, next_state, self_state):
    """
    「x,yは現在地を維持したまま、深度[m]だけを絶対値で目標にする」Stateのハンドラを生成する。
    DESCEND_TO_7M専用(「7mまで潜る」は起動時の深度に依存しない絶対深度指定のはずなので、
    _make_relative_dive_handlerの「今いる場所から+7m」とは意味が異なる)。
    """
    def target_fn(ctx: MissionContext) -> np.ndarray:
        base = ctx.position if ctx.position is not None else np.zeros(3)
        return np.array([base[0], base[1], depth_m])

    def handler(ctx: MissionContext) -> State:
        result = _move_toward(ctx, target_fn)
        if result.emergency:
            return State.EMERGENCY
        if result.reached:
            return next_state
        return self_state

    return handler


# ---- ここから下、_image_processing_available_stub と _mic_data_from_above_stub は
# 今回の対象外(YOLO/ハイドロフォンの相対値変換とは別の話)なので、まだスタブのまま ----

def _image_processing_available_stub():
    return True  # TODO: 誰が/どう判定するか未定


def _mic_data_from_above_stub():
    return False  # TODO Step5以降


# ---- Stateハンドラ ----

def handle_init(ctx: MissionContext) -> State:
    if _time_in_state(ctx) < params.INIT_WAIT_SEC:
        return State.INIT
    return State.DESCEND_TO_7M


def handle_search_hydrophone(ctx: MissionContext) -> State:
    if _time_in_state(ctx) > params.SEARCH_TIMEOUT_SEC:
        return State.SURFACE

    bearing = perception.get_latest_hydrophone_bearing()
    quat = ctx.quaternion if ctx.quaternion is not None else np.array([1.0, 0.0, 0.0, 0.0])

    # TODO(要ハードウェア仕様確認): 不等号の向きは "対象がハイドロフォンより深い(HYDRO_DIVEで
    # さらに+2.5m潜る設計と整合)" と仮定して pitch_deg > THRESHOLD (見下ろす角度が大きい=近い) に
    # している。ただしperception.HydrophoneBearingの「正=対象が下方向」の定義と実機の取り付け向きに
    # 完全に依存するため、実機で必ず符号を検証すること。逆であれば `<` に戻す。
    if bearing.pitch_deg > HYDROPHONE_PITCH_THRESHOLD_DEG:
        # 閾値を超えた瞬間の推定座標を「発見した対象の絶対座標」として記録する。
        # 以降のHOLD_TARGET/HYDRO_DIVE/HYDRO_APPROACHはこれを参照するので、
        # ここで一度も呼ばれなければ後続Stateは目標を持てない。
        base = ctx.position if ctx.position is not None else np.zeros(3)
        body_offset = perception.bearing_to_body_offset(bearing)
        ctx.discovered_target = perception.body_offset_to_ned(base, quat, body_offset)
        ctx.attack_retry_started_at = ctx.elapsed_time  # マイク確認待ちループのタイムアウト起点
        if _image_processing_available_stub():
            return State.VISUAL_DIVE
        return State.HOLD_TARGET

    # BingerSearchは毎tick方角を再計算する探索なので、target_positionのキャッシュは使わない。
    # 「現在位置からのオフセット」が欲しいので、回転だけ適用し原点はゼロのまま渡す。
    body_offset = perception.bearing_to_body_offset(bearing)
    dx, dy, dz = perception.body_offset_to_ned(np.zeros(3), quat, body_offset)
    result = _run_control_tick(ctx, dx, dy, dz)
    if result.emergency:
        return State.EMERGENCY
    return State.SEARCH_HYDROPHONE


def handle_visual_check_mic(ctx: MissionContext) -> State:
    if _mic_data_from_above_stub():
        return State.SURFACE
    return State.VISUAL_DETECT


def handle_visual_detect(ctx: MissionContext) -> State:
    import time
    detection = perception.get_latest_yolo_detection()
    if perception.is_yolo_detection_usable(detection, now=time.time()):
        base = ctx.position if ctx.position is not None else np.zeros(3)
        quat = ctx.quaternion if ctx.quaternion is not None else np.array([1.0, 0.0, 0.0, 0.0])
        body_offset = (detection.x_m, detection.y_m, detection.z_m)
        ctx.discovered_target = perception.body_offset_to_ned(base, quat, body_offset)
        return State.VISUAL_ATTACK
    return State.VISUAL_DETECT


def _visual_attack_target(ctx: MissionContext) -> np.ndarray:
    # VISUAL_DETECTが記録したdiscovered_targetをそのまま使う。
    # ここがNoneなのは設計上ありえない(handle_visual_detectが必ず先に設定する)ため、
    # 万一Noneなら現在位置をそのまま返して足止めする(前進しない=安全側に倒す)。
    if ctx.discovered_target is not None:
        return ctx.discovered_target
    return ctx.position if ctx.position is not None else np.zeros(3)


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
    return ctx.position if ctx.position is not None else np.zeros(3)


def handle_hydro_approach(ctx: MissionContext) -> State:
    result = _move_toward(ctx, _hydro_approach_target)
    if result.emergency:
        return State.EMERGENCY
    if result.reached:
        return State.HYDRO_NEAR_SURFACE
    return State.HYDRO_APPROACH


def handle_hydro_check_mic(ctx: MissionContext) -> State:
    if _mic_data_from_above_stub():
        return State.SURFACE
    return State.HOLD_TARGET


def handle_return_home(ctx: MissionContext) -> State:
    return State.DONE  # canvas注記通り、できなければ何もせずDONEでよい


handle_visual_dive = _make_relative_dive_handler((0.0, 0.0, 2.5), State.VISUAL_CHECK_MIC, State.VISUAL_DIVE)
handle_hydro_dive = _make_relative_dive_handler((0.0, 0.0, 2.5), State.HYDRO_APPROACH, State.HYDRO_DIVE)
handle_visual_post_check = _make_relative_dive_handler((0.0, 0.0, -0.5), State.VISUAL_CHECK_MIC, State.VISUAL_POST_CHECK)
handle_hydro_near_surface = _make_relative_dive_handler((0.0, 0.0, -0.5), State.HYDRO_CHECK_MIC, State.HYDRO_NEAR_SURFACE)
handle_surface = _make_relative_dive_handler((0.0, 0.0, -4.0), State.RETURN_HOME, State.SURFACE)

STATE_HANDLERS = {
    State.INIT: handle_init,
    State.DESCEND_TO_7M: _make_absolute_depth_dive_handler(7.0, State.SEARCH_HYDROPHONE, State.DESCEND_TO_7M),
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


_previous_state = None  # controlfunction._was_holdingと同じ考え方の、モジュールレベル永続変数


def mission_tick(ctx: MissionContext, state: State) -> State:
    """現在のStateに対応するハンドラを1回呼び、次のStateを返す。
    DONE/EMERGENCYなどハンドラを持たない終端Stateはそのまま返す(呼び出し側でループを止める)。

    Stateが前回の呼び出しから変わっていた場合、_time_in_state()の基準時刻と
    target_positionをここで一括リセットする(個々のハンドラがバラバラに管理しない)。
    """
    global _previous_state
    if state != _previous_state:
        ctx._state_entered_at = ctx.elapsed_time
        ctx.target_position = None
        _previous_state = state

    handler = STATE_HANDLERS.get(state)
    if handler is None:
        return state
    return handler(ctx)
