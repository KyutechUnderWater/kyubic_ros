"""ピンガー探査ミッションのFSM(ステートマシン)本体。

ROSのI/O(購読・配信・サービス・PID)は pinger_search_node.PingerSearchNode が持ち、
このクラスは「今どのstepに居て、何を目標にし、いつ次へ進むか」だけを担当する。
1ステート = 1メソッド(_tick_xxx)で、docstringに [step番号] 動作 / 遷移条件 を書く。

ステート遷移図(番号はミッション仕様のstep):

  WAIT_START ─(トリガ)→ START_DELAY(30秒待機)
    ─(Yaw1記憶)→ ARMING ─(ARM成功)→ INITIAL_DASH                    … step1
    → DIVE(4mへ潜航)                                                 … step2
    → PINGER_APPROACH(観測→旋回→再確認→前進の繰返し)                 … step3
    → FINAL_DESCEND(0.5m刻みで降下、pitchが正面に来るまで)           … step4
    → RISE_ALIGN(charge_rise_m浮上 → その深度でヨー合わせ)            … step4'
      (ピンガーは目標物の下側に付いているため、突進前に少し上がる)
    ├ enable_vision=true:
    │   → VISION_ALIGN(画処理ON、YOLOで中心合わせ)                   … step5
    │   → CHARGE(突進)                                               … step6
    └ enable_vision=false(画処理なし):
        → CHARGE(数秒前進)                                            … step5-6の代替
    → RETREAT_GOTO(4m/Yaw1へ) → RETREAT_BACK(5秒後進)                … step7
    → SURFACE_CHECK ─(ピンFINAL_DESCENDガーが上)→ ASCEND_FINISH ─→ DISARMING → FINISHED
                    └(まだ下)→ PINGER_APPROACH へ戻る                … step8
  ※ どのstepでも開始9分超過で ABORT_ASCEND(強制浮上) → DISARMING     … step9
  ※ 1動作(ダッシュ・潜航・前進・突進・後進など)が終わるたびに PAUSE を挟み、
    action_pause_s 秒だけ推進を止めて機体を落ち着かせる(深度・ヨーは保持)。
"""

import math
from enum import Enum, auto

from .pinger_filter import robust_circular_mean_deg, wrap_deg


class State(Enum):
    WAIT_START = auto()
    START_DELAY = auto()
    ARMING = auto()
    INITIAL_DASH = auto()
    DIVE = auto()
    PINGER_APPROACH = auto()
    FINAL_DESCEND = auto()
    RISE_ALIGN = auto()
    VISION_ALIGN = auto()
    CHARGE = auto()
    PAUSE = auto()  # 動作間の小休止(次のステートは_pause_nextに保持)
    RETREAT_GOTO = auto()
    RETREAT_BACK = auto()
    SURFACE_CHECK = auto()
    ASCEND_FINISH = auto()
    ABORT_ASCEND = auto()
    DISARMING = auto()
    FINISHED = auto()


# step9のミッションタイムアウト(9分)監視の対象。浮上・終了系は対象外。
TIMEOUT_MONITORED_STATES = {
    State.ARMING,
    State.INITIAL_DASH,
    State.DIVE,
    State.PINGER_APPROACH,
    State.FINAL_DESCEND,
    State.RISE_ALIGN,
    State.VISION_ALIGN,
    State.CHARGE,
    State.PAUSE,
    State.RETREAT_GOTO,
    State.RETREAT_BACK,
    State.SURFACE_CHECK,
}

# 深度/ヨー制御を行う(=imu/depthの鮮度が必須の)ステート。
SENSOR_REQUIRED_STATES = TIMEOUT_MONITORED_STATES | {State.ASCEND_FINISH, State.ABORT_ASCEND}

# 各ステートの上限滞在時間(ハング防止)。条件待ちのステートが対象で、値は
# 使うパラメータ名。時間で必ず終わるステート(ダッシュ/突進/後進/PAUSE等)は対象外。
# 超過時の抜け先は _check_state_timeout() に記述。
STATE_TIMEOUT_PARAM = {
    State.ARMING: "state_timeout_s",
    State.DIVE: "state_timeout_s",
    State.PINGER_APPROACH: "approach_timeout_s",  # 移動距離があるので別途長めに
    State.FINAL_DESCEND: "state_timeout_s",
    State.RISE_ALIGN: "state_timeout_s",
    State.VISION_ALIGN: "state_timeout_s",
    State.RETREAT_GOTO: "state_timeout_s",
    State.SURFACE_CHECK: "state_timeout_s",
    State.ASCEND_FINISH: "state_timeout_s",
    State.ABORT_ASCEND: "state_timeout_s",
    State.DISARMING: "state_timeout_s",
}


class MissionFsm:
    """ミッション進行の状態と目標値(深度・ヨー)を管理する。

    node(PingerSearchNode)からは毎制御周期 tick(now, dt) が呼ばれ、
    ピンガーの採用サンプルごとに on_pinger_accepted() が呼ばれる。
    """

    def __init__(self, node) -> None:
        self._n = node
        self.state = State.WAIT_START
        self._state_entered = 0.0
        self._mission_start: float | None = None
        # 目標値。深度は正=深い[m]、ヨーは方位[deg](IMU orient.z基準)
        self.yaw1_deg: float | None = None  # ミッション開始時の機首方位(step7で使用)
        self.yaw_target_deg: float | None = None
        self.depth_target_m: float | None = None
        # 到達判定・サブモード管理
        self._settle_since: float | None = None  # 目標到達がこの時刻から継続中
        self._forward_until: float | None = None  # step3: 前進サブモードの終了時刻
        self._wait_pinger_seq = 0  # step8: このseqより新しいピンガー更新を待つ
        self._center_hold_since: float | None = None  # step6: 中央に入った時刻
        self._pause_next: State | None = None  # PAUSE明けに入るステート
        self._pause_until: float | None = None  # step3/track: 前進後の小休止の終了時刻
        # step3のサブモード("collect"/"turn"/"recheck"/"forward"/"rest")と
        # 方位平均用のサンプル(絶対方位[deg]、機首+ピンガー相対yaw)
        self._chase_mode = "collect"
        self._bearing_samples: list[float] = []
        self._front_since: float | None = None  # step4: 正面条件がこの時刻から連続成立中
        self._vision_seen = False  # step5: 直前のtickでYOLO検出を追えていたか
        self._above_since: float | None = None  # ピンガー「上」判定がこの時刻から連続成立中
        self._rise_done = False  # step4': charge_rise_m の浮上が完了したか
        self._charge_elapsed = 0.0  # step6: 実際に前進した累計時間
        self._lifting = False  # step6: 突進終了後、前進しながら浮上する段階に入ったか
        self._lift_trial_elapsed = 0.0  # step6: 通常の深度PIDで浮上を試みている時間
        self._lift_forced = False  # step6: 通常PIDで浮上できず、最大推力に切り替えたか
        self._lift_elapsed = 0.0  # step6: 最大推力フェーズに入ってからの経過時間

    # ------------------------------------------------------------ 外部イベント

    def on_pinger_accepted(self) -> None:
        """ピンガーの採用サンプルが来た時にnodeから呼ばれる。

        step3では観測サブモード(collect/recheck)中のみ、絶対方位
        「現在の機首 + ピンガー相対yaw」をサンプルとして貯める(数回分の外れ値除去
        平均に正面を合わせるため。旋回・前進中のサンプルは使わない)。
        FINAL_DESCENDではヨー目標を更新しない: 伏角が深い(ほぼ真下)区間は水平方向の
        誤差がわずかでも方位角が大きく振れ、単発サンプル追従だとヨーが暴れるため。
        深度ステップの遷移条件は伏角のみで判定しヨーには依存せず、本番のヨー合わせは
        この後のRISE_ALIGNで複数観測の外れ値除去平均により行うので、FINAL_DESCEND中は
        PINGER_APPROACHで最後に確定した向きをそのままPID保持すればよい。
        """
        n = self._n
        if n.yaw_deg is None:
            return
        if self.state in (State.PINGER_APPROACH, State.RISE_ALIGN) or (
            self.state is State.VISION_ALIGN and not self._vision_seen
        ):
            # step4'のヨー合わせ、step5でYOLO未検出の間も同じ観測を使う
            if self._chase_mode in ("collect", "recheck"):
                self._bearing_samples.append(wrap_deg(n.yaw_deg + n.pinger_yaw_deg))

    # ------------------------------------------------------------ メインループ

    def tick(self, now: float, dt: float) -> None:
        n = self._n

        # [step9] ミッションタイムアウト: 9分超過で強制浮上へ
        if (
            self._mission_start is not None
            and self.state in TIMEOUT_MONITORED_STATES
            and now - self._mission_start > n.p("mission_timeout_s")
        ):
            n.get_logger().warning("mission timeout: forcing ascend to finish depth")
            n.publish_image_enable(False)
            self._transition(now, State.ABORT_ASCEND)

        # ステートタイムアウト: 条件待ちがハングしないよう、上限を超えたら強制的に先へ進む
        self._check_state_timeout(now)

        # センサ死活監視: 制御に必要なステートでimu/depthが古ければ中立を出して何もしない
        if self.state in SENSOR_REQUIRED_STATES and n.sensors_stale(now):
            n.get_logger().warning(
                "imu/depth stale: sending neutral wrench", throttle_duration_sec=2.0
            )
            n.publish_wrench(0.0, 0.0, 0.0)
            return

        handler = {
            State.WAIT_START: self._tick_wait_start,
            State.START_DELAY: self._tick_start_delay,
            State.ARMING: self._tick_arming,
            State.INITIAL_DASH: self._tick_initial_dash,
            State.DIVE: self._tick_dive,
            State.PINGER_APPROACH: self._tick_pinger_approach,
            State.FINAL_DESCEND: self._tick_final_descend,
            State.RISE_ALIGN: self._tick_rise_align,
            State.VISION_ALIGN: self._tick_vision_align,
            State.CHARGE: self._tick_charge,
            State.PAUSE: self._tick_pause,
            State.RETREAT_GOTO: self._tick_retreat_goto,
            State.RETREAT_BACK: self._tick_retreat_back,
            State.SURFACE_CHECK: self._tick_surface_check,
            State.ASCEND_FINISH: self._tick_ascend,
            State.ABORT_ASCEND: self._tick_ascend,
            State.DISARMING: self._tick_disarming,
            State.FINISHED: self._tick_finished,
        }[self.state]
        handler(now, dt)

    def _check_state_timeout(self, now: float) -> None:
        """[全制御共通のタイムアウト] 条件待ちステートが上限時間を超えたら強制遷移する。

        通常の遷移条件(深度到達・ピンガー正面・中央合わせ等)が満たされないまま
        滞在時間が state_timeout_s (PINGER_APPROACHのみ approach_timeout_s) を超えた場合、
        「そのステートが本来進むはずだった先」へ進める。浮上・DISARM系は先へ進める方が
        安全側(必ずDISARM/終了に到達する)。
        """
        n = self._n
        param = STATE_TIMEOUT_PARAM.get(self.state)
        if param is None or self._in_state_for(now) <= n.p(param):
            return
        n.get_logger().warning(f"state timeout ({n.p(param):.0f}s): {self.state.name} -> forcing next")
        s = self.state
        if s is State.ARMING:
            # ARMできない=推力も出ない。安全に終了する
            n.get_logger().error("arming timed out: aborting mission")
            self._transition(now, State.FINISHED)
        elif s is State.DIVE:
            self._enter_approach(now)  # 目標深度に届かなくても探査は開始できる
        elif s is State.PINGER_APPROACH:
            self._transition(now, State.FINAL_DESCEND)
        elif s is State.FINAL_DESCEND:
            if n.p("enable_vision", bool):
                self._start_pause(now, State.VISION_ALIGN)
            else:
                # 正面を確認できないままのsurgeはしない: そのまま離脱へ
                self._start_pause(now, State.RETREAT_GOTO)
        elif s is State.RISE_ALIGN:
            # ヨー合わせが完了しないまま突進はしない: そのまま離脱へ
            self._start_pause(now, State.RETREAT_GOTO)
        elif s is State.VISION_ALIGN:
            # 中央合わせできないまま突進はしない: 画処理を止めて離脱へ
            n.publish_image_enable(False)
            self._start_pause(now, State.RETREAT_GOTO)
        elif s is State.RETREAT_GOTO:
            self._start_pause(now, State.RETREAT_BACK)  # 整定しきらなくても後進へ
        elif s is State.SURFACE_CHECK:
            # 判定材料が集まらない場合は「上に来た」扱いで浮上終了(9分棄権より前向きに終わる)
            self._transition(now, State.ASCEND_FINISH)
        elif s in (State.ASCEND_FINISH, State.ABORT_ASCEND):
            n.retry_arm(False, now)
            self._transition(now, State.DISARMING)  # 浮上しきらなくても必ずDISARMへ
        elif s is State.DISARMING:
            n.get_logger().error("disarm not confirmed: finish anyway. DISARM MANUALLY!")
            self._transition(now, State.FINISHED)

    # ----------------------------------------------------------- 各ステート

    def _tick_wait_start(self, now: float, dt: float) -> None:
        """[開始待ち] mission_start_trigger(リードスイッチ)が true になるまで中立で待つ。

        遷移: トリガ受信 かつ IMU受信済み → その瞬間のヨーをYaw1として記憶し、
              START_DELAY(投入・退避のための待機)へ。IMUがまだ無ければ届くまで待つ。
        """
        n = self._n
        n.publish_wrench(0.0, 0.0, 0.0)
        if not n.start_triggered:
            return
        if n.yaw_deg is None:
            n.get_logger().warning(
                "start triggered but no IMU yaw yet; waiting", throttle_duration_sec=2.0
            )
            return
        self.yaw1_deg = n.yaw_deg
        n.get_logger().info(
            f"start triggered: Yaw1 = {self.yaw1_deg:.1f} deg; "
            f"waiting {n.p('start_delay_s'):.0f} s before mission"
        )
        self._transition(now, State.START_DELAY)

    def _tick_start_delay(self, now: float, dt: float) -> None:
        """[開始前待機] トリガ後 start_delay_s(30秒)だけ中立のまま待つ。

        遷移: 時間経過 → ミッション開始時刻(9分タイムアウトの起点)を打ってARMINGへ。
              Yaw1はWAIT_START側でトリガ受信時に記憶済み。
        """
        n = self._n
        n.publish_wrench(0.0, 0.0, 0.0)
        if self._in_state_for(now) < n.p("start_delay_s"):
            return
        self._mission_start = now
        n.get_logger().info(f"start delay elapsed: mission start (Yaw1 = {self.yaw1_deg:.1f} deg)")
        n.retry_arm(True, now)
        self._transition(now, State.ARMING)

    def _tick_arming(self, now: float, dt: float) -> None:
        """[ARM] set_armed(true) を呼ぶ。失敗/未応答なら3秒ごとにリトライ。

        遷移: vehicle_stateがarmed、またはサービス応答success → INITIAL_DASH。
        """
        n = self._n
        n.publish_wrench(0.0, 0.0, 0.0)
        if n.vehicle_armed or n.poll_arm_response() is True:
            self._transition(now, State.INITIAL_DASH)
            return
        n.retry_arm(True, now)

    def _tick_initial_dash(self, now: float, dt: float) -> None:
        """[step1] RF x = dash_force_n を dash_duration_s 秒印加(水面ダッシュ)。

        深度制御なし。直進性のためヨーだけYaw1に保持する。
        遷移: 時間経過 → 深度目標4m(cruise_depth_m)をセットしてDIVEへ。
        """
        n = self._n
        self.yaw_target_deg = self.yaw1_deg
        if self._in_state_for(now) >= n.p("dash_duration_s"):
            self._start_pause(now, State.DIVE)
            n.publish_wrench(0.0, 0.0, self._yaw_torque(dt))
            return
        n.publish_wrench(n.p("dash_force_n"), 0.0, self._yaw_torque(dt))

    def _tick_dive(self, now: float, dt: float) -> None:
        """[step2] cruise_depth_m(4m)まで潜航し保持。ヨーはYaw1のまま。

        遷移: 深度が許容内にsettle → 小休止を挟んでPINGER_APPROACHへ。
        """
        n = self._n
        self.depth_target_m = n.p("cruise_depth_m")
        n.publish_wrench(0.0, self._depth_force(dt), self._yaw_torque(dt))
        if self._settled(now, check_yaw=False):
            self._start_pause(now, State.PINGER_APPROACH)

    def _tick_pinger_approach(self, now: float, dt: float) -> None:
        """[step3] 「観測→旋回→停止して再確認→前進」を繰返しピンガーへ近づく。深度4m保持。

        サブモードの詳細は _chase_wrench() を参照(数回観測の外れ値除去平均に正面を
        合わせ、前進中はヨーを制御しない)。
        遷移: ピンガー伏角 >= pinger_below_pitch_deg (ほぼ真下) → FINAL_DESCEND。
              ピンガー「上」判定が pinger_above_hold_s (10秒)連続 → 通り過ぎ確定、
              ミッション完了として ASCEND_FINISH(浮上終了)。
        """
        n = self._n
        if self._pinger_above_held(now):
            n.get_logger().info(
                "pinger stayed above during approach: mission complete, ascending"
            )
            self._transition(now, State.ASCEND_FINISH)
            n.publish_wrench(0.0, self._depth_force(dt), self._yaw_torque(dt))
            return
        if (
            n.pinger_depression_deg is not None
            and n.pinger_depression_deg >= n.p("pinger_below_pitch_deg")
        ):
            self._transition(now, State.FINAL_DESCEND)
            n.publish_wrench(0.0, self._depth_force(dt), self._yaw_torque(dt))
            return

        fx, tq = self._chase_wrench(now, dt)
        n.publish_wrench(fx, self._depth_force(dt), tq)

    def _tick_final_descend(self, now: float, dt: float) -> None:
        """[step4] 4m保持をやめ、depth_step_m(0.5m)刻みで降下しつつピンガーを正面へ。

        ヨー目標はPINGER_APPROACHで最後に確定した向きのまま更新せずPID保持する
        (on_pinger_accepted参照。伏角が深い区間は方位角がノイズで大きく振れるため)。
        深度は「現ステップの深度に到達してsettle→次のステップへ0.5m下げる」を、
        伏角が正面範囲に入るまで繰返す(max_depth_mで頭打ち)。
        遷移: |伏角| <= pinger_front_pitch_deg が pinger_front_confirm_s 連続で成立
              (=確実に正面ピッチ) → 深度目標を charge_rise_m 上げて、小休止を挟んで
              RISE_ALIGN(浮上→ヨー合わせ)へ。
              ※ ピンガーは目標物の下側に付いているため、ヨー合わせと突進は
                少し浮上した深度で行う。yawの最終合わせはRISE_ALIGN側。
        """
        n = self._n
        depression = n.pinger_depression_deg
        front_pitch = n.p("pinger_front_pitch_deg")

        front_ok = depression is not None and abs(depression) <= front_pitch
        if not front_ok:
            self._front_since = None
        elif self._front_since is None:
            self._front_since = now
        elif now - self._front_since >= n.p("pinger_front_confirm_s"):
            self._front_since = None
            self.depth_target_m = max(
                self.depth_target_m - n.p("charge_rise_m"), n.p("finish_depth_m")
            )
            n.get_logger().info(
                f"front pitch confirmed: rising to {self.depth_target_m:.1f} m before align"
            )
            self._start_pause(now, State.RISE_ALIGN)
            n.publish_wrench(0.0, self._depth_force(dt), self._yaw_torque(dt))
            return

        # まだピンガーが正面より下 → 深度ステップを1段下げる(到達+settle後)
        still_below = depression is not None and depression > front_pitch
        if still_below and self._settled(now, check_yaw=False):
            new_target = min(self.depth_target_m + n.p("depth_step_m"), n.p("max_depth_m"))
            if new_target > self.depth_target_m:
                n.get_logger().info(f"descend step: depth target -> {new_target:.1f} m")
            self.depth_target_m = new_target
            self._settle_since = None

        n.publish_wrench(0.0, self._depth_force(dt), self._yaw_torque(dt))

    def _tick_rise_align(self, now: float, dt: float) -> None:
        """[step4'] charge_rise_m 浮上し、その深度でヨーをピンガーに合わせる。

        ピンガーが目標物の下側に付いているため、ピッチ正面の深度から少し上がって
        から突進する。流れ:
          1. FINAL_DESCEND終了時に上げられた深度目標に settle するまで待つ
          2. step3と同じ「停止観測→旋回→再確認」でヨーを合わせる(前進はしない)
        遷移: ヨー合わせ完了 → enable_vision=true: VISION_ALIGN / false: CHARGE。
              (完了判定は _chase_wrench 内のRISE_ALIGN特例で行う)
        """
        n = self._n
        if not self._rise_done:
            n.publish_wrench(0.0, self._depth_force(dt), self._yaw_torque(dt))
            if self._settled(now, check_yaw=False):
                self._rise_done = True
                self._reset_chase()  # 浮上後の新しい観測からヨー合わせを始める
                self._settle_since = None
            return
        _, tq = self._chase_wrench(now, dt)
        n.publish_wrench(0.0, self._depth_force(dt), tq)  # このステートでは前進しない

    def _tick_vision_align(self, now: float, dt: float) -> None:
        """[step5-6] 画処理ON(ライトも連動点灯)。検出の有無で動作を切り替える:

        - YOLOが検出できている間: 水平正規化偏差(右+)に比例した速度でヨー目標のみ
          なぞらせ、PIDで画面中央(左右)へ合わせる。
          深度は画処理では動かさず、RISE_ALIGNで決めた深度を保持し続ける。
        - 未検出(buoy_timeout_s途絶)の間: ピンガーアプローチと同じ
          「観測→旋回→再確認→前進」(_chase_wrench)でピンガーの真上に留まる/近づく。
          検出が戻り次第、中心合わせに切り替える。
        遷移: 検出中に|水平偏差| <= center_tolerance が center_hold_s 続いたら → CHARGE。
        """
        n = self._n
        n.publish_image_enable(True)
        buoy = n.buoy
        fresh = (
            buoy is not None
            and now - n.buoy_time <= n.p("buoy_timeout_s")
            and buoy.detected
            and math.isfinite(buoy.normalized_horizontal_offset)
        )

        if not fresh:
            # YOLO未検出: 音響でピンガーを追う(検出が戻るまで)
            if self._vision_seen:
                self._vision_seen = False
                self._reset_chase()
                n.reset_yaw_pid()
            self._center_hold_since = None
            fx, tq = self._chase_wrench(now, dt)
            n.publish_wrench(fx, self._depth_force(dt), tq)
            return

        if not self._vision_seen:
            # 検出復帰: chaseを止め、現在の機首を起点に中心合わせを始める
            self._vision_seen = True
            self.yaw_target_deg = n.yaw_deg
            n.reset_yaw_pid()

        h = buoy.normalized_horizontal_offset  # 正=右
        tol = n.p("center_tolerance")
        centered = abs(h) <= tol  # 深度は画処理で合わせない: 左右のみ判定・調整
        if not centered:
            self.yaw_target_deg = wrap_deg(
                self.yaw_target_deg + n.p("vision_yaw_rate_deg_s") * h * dt
            )

        if centered:
            if self._center_hold_since is None:
                self._center_hold_since = now
            elif now - self._center_hold_since >= n.p("center_hold_s"):
                self._transition(now, State.CHARGE)
                n.publish_wrench(
                    n.p("charge_force_n"), self._depth_force(dt), self._yaw_torque(dt)
                )
                return
        else:
            self._center_hold_since = None

        n.publish_wrench(0.0, self._depth_force(dt), self._yaw_torque(dt))

    def _tick_charge(self, now: float, dt: float) -> None:
        """[step6 / 画処理なし時はstep5-6の代替] charge_force_n で計 charge_duration_s 秒
        前進(突進)する。深度・ヨーは保持。

        遷移: 前進の累計が charge_duration_s → 深度目標を post_charge_lift_m (0.35m)浅くし、
              charge_force_n で前進を続けながらまず通常の深度PIDで浮上を試みる(ブイに
              引っかかっていなければこれで普通に上がるはず)。post_charge_lift_trial_s
              以内に到達できなければ「ブイに引っかかった」と判定し、post_charge_lift_force_n
              (深度PIDではなく固定の最大推力)に切り替えて post_charge_lift_duration_s 秒
              引っ張って外す。どちらの経路でも、完了後は画処理OFF、小休止を挟んでRETREAT_GOTOへ。
              ピンガー「上」判定が pinger_above_hold_s (10秒)連続 → 通り過ぎたと
              みなして突進を中断。深度目標をcruiseに戻し、ヨー目標もYaw1に戻して、
              RETREAT_GOTOを飛ばして直接RETREAT_BACK(後進)へ → その後SURFACE_CHECK。
        """
        n = self._n
        if self._pinger_above_held(now):
            n.get_logger().info(
                "pinger stayed above during charge: aborting, retreat and recheck"
            )
            n.publish_image_enable(False)
            self.depth_target_m = n.p("cruise_depth_m")
            self.yaw_target_deg = self.yaw1_deg
            self._start_pause(now, State.RETREAT_BACK)
            n.publish_wrench(0.0, self._depth_force(dt), self._yaw_torque(dt))
            return

        # 前進(突進)。実前進時間だけを積算する
        self._charge_elapsed += dt
        if self._charge_elapsed < n.p("charge_duration_s"):
            n.publish_wrench(n.p("charge_force_n"), self._depth_force(dt), self._yaw_torque(dt))
            return

        # 突進時間終了: 深度目標をpost_charge_lift_m浅くする(まだの場合のみ)
        if not self._lifting:
            self._lifting = True
            self.depth_target_m = max(
                self.depth_target_m - n.p("post_charge_lift_m"), n.p("finish_depth_m")
            )
            n.get_logger().info(
                f"charge complete: trying to rise to {self.depth_target_m:.1f} m "
                "with normal depth PID while still moving forward"
            )

        if not self._lift_forced:
            # フェーズ1: 前進を続けつつ、まず通常の深度PIDで浮上を試みる。
            # 到達できればブイに引っかかっていない(=既に外れている/そもそも引っかけて
            # いない)とみなし、そのまま次へ進む。
            if self._settled(now, check_yaw=False):
                n.get_logger().info("rise succeeded with normal depth PID: proceeding")
                n.publish_image_enable(False)
                self._start_pause(now, State.RETREAT_GOTO)
                n.publish_wrench(0.0, self._depth_force(dt), self._yaw_torque(dt))
                return
            self._lift_trial_elapsed += dt
            if self._lift_trial_elapsed < n.p("post_charge_lift_trial_s"):
                n.publish_wrench(n.p("charge_force_n"), self._depth_force(dt), self._yaw_torque(dt))
                return
            # 通常の深度PIDでは浮上できなかった: ブイに引っかかったと判定し、
            # 最大推力フェーズへ切り替える。
            self._lift_forced = True
            self._settle_since = None
            n.get_logger().info(
                "rise stalled with normal depth PID: assuming caught on the buoy, "
                "switching to max thrust"
            )

        # フェーズ2: 深度PIDではなく post_charge_lift_force_n を固定で出し続けて引っ張る。
        self._lift_elapsed += dt
        if self._lift_elapsed < n.p("post_charge_lift_duration_s"):
            n.publish_wrench(
                n.p("charge_force_n"), -n.p("post_charge_lift_force_n"), self._yaw_torque(dt)
            )
            return

        n.publish_image_enable(False)
        self._start_pause(now, State.RETREAT_GOTO)
        n.publish_wrench(0.0, self._depth_force(dt), self._yaw_torque(dt))

    def _tick_retreat_goto(self, now: float, dt: float) -> None:
        """[step7前半] 深度4m・機首Yaw1へ整定させる。

        遷移: 深度・ヨーともに許容内にsettle → 小休止を挟んでRETREAT_BACK。
        """
        n = self._n
        n.publish_image_enable(False)
        self.depth_target_m = n.p("cruise_depth_m")
        self.yaw_target_deg = self.yaw1_deg
        n.publish_wrench(0.0, self._depth_force(dt), self._yaw_torque(dt))
        if self._settled(now, check_yaw=True):
            self._start_pause(now, State.RETREAT_BACK)

    def _tick_retreat_back(self, now: float, dt: float) -> None:
        """[step7後半] retreat_force_n で retreat_duration_s(5秒)後進して停止。

        遷移: 時間経過 → ピンガーフィルタをリセットし(古い観測を捨てて、この場からの
              観測だけで判定するため)、小休止を挟んでSURFACE_CHECKへ。
        """
        n = self._n
        if self._in_state_for(now) >= n.p("retreat_duration_s"):
            n.reset_pinger_filter()
            self._wait_pinger_seq = n.pinger_seq
            self._start_pause(now, State.SURFACE_CHECK)
            n.publish_wrench(0.0, self._depth_force(dt), self._yaw_torque(dt))
            return
        n.publish_wrench(-n.p("retreat_force_n"), self._depth_force(dt), self._yaw_torque(dt))

    def _tick_surface_check(self, now: float, dt: float) -> None:
        """[step8] 深度4m保持のままピンガーの高さを再観測して判定する。

        surface_check_duration_s 以上観測し、かつフィルタ窓が埋まるだけの新規サンプル
        (リセット直後の単発外れ値で誤判定しないため)が来てから:
          - 伏角 <= -pinger_above_pitch_deg (ピンガーがロボットより上) → ASCEND_FINISH
          - まだ下 → PINGER_APPROACH へ戻る(深度4mは保持済みなのでstep3から再開)
        """
        n = self._n
        n.publish_wrench(0.0, self._depth_force(dt), self._yaw_torque(dt))
        if self._in_state_for(now) < n.p("surface_check_duration_s"):
            return
        if n.pinger_seq < self._wait_pinger_seq + n.p("pinger_filter_window", int):
            n.get_logger().warning(
                "not enough pinger samples during surface check; keep waiting",
                throttle_duration_sec=5.0,
            )
            return
        if n.pinger_depression_deg <= -n.p("pinger_above_pitch_deg"):
            n.get_logger().info("pinger is above the robot: mission complete, ascending")
            self._transition(now, State.ASCEND_FINISH)
        else:
            n.get_logger().info("pinger still below: restarting approach")
            self._enter_approach(now)

    def _tick_ascend(self, now: float, dt: float) -> None:
        """[step8終了/step9強制] finish_depth_m(1m)まで浮上する(ヨーは現目標を保持)。

        遷移: 深度が許容内にsettle → set_armed(false)を送ってDISARMINGへ。
        """
        n = self._n
        n.publish_image_enable(False)
        self.depth_target_m = n.p("finish_depth_m")
        n.publish_wrench(0.0, self._depth_force(dt), self._yaw_torque(dt))
        if self._settled(now, check_yaw=False):
            n.retry_arm(False, now)
            self._transition(now, State.DISARMING)

    def _tick_disarming(self, now: float, dt: float) -> None:
        """[終了処理] set_armed(false)の完了を待つ。失敗/未応答なら3秒ごとにリトライ。

        遷移: vehicle_stateがdisarm、またはサービス応答success → FINISHED。
        """
        n = self._n
        n.publish_wrench(0.0, 0.0, 0.0)
        if not n.vehicle_armed or n.poll_arm_response() is True:
            n.get_logger().info("disarmed: mission finished")
            self._transition(now, State.FINISHED)
            return
        n.retry_arm(False, now)

    def _tick_pause(self, now: float, dt: float) -> None:
        """[小休止] 1動作終了ごとに action_pause_s 秒だけ推進を止め機体を落ち着かせる。

        前進力は出さず、深度・ヨーは現在の目標を保持(目標未設定なら完全中立)。
        遷移: 時間経過 → _pause_next のステートへ。PINGER_APPROACHへ入る場合は
              「休止後の新しい観測」から動き出すようサブモードをリセットする。
        """
        n = self._n
        n.publish_wrench(0.0, self._depth_force(dt), self._yaw_torque(dt))
        if self._in_state_for(now) < n.p("action_pause_s"):
            return
        next_state = self._pause_next
        self._pause_next = None
        if next_state in (State.PINGER_APPROACH, State.RISE_ALIGN, State.VISION_ALIGN):
            self._reset_chase()  # 休止後の新しい観測から動き出す
            self._vision_seen = False
            self._rise_done = False
        self._transition(now, next_state)

    def _tick_finished(self, now: float, dt: float) -> None:
        """[終了] 以後は中立wrenchを出し続ける(heartbeatはnode側で継続)。"""
        self._n.publish_wrench(0.0, 0.0, 0.0)

    # ------------------------------------------------------------- 内部補助

    def _transition(self, now: float, new_state: State) -> None:
        self._n.get_logger().info(f"state: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self._state_entered = now
        self._settle_since = None
        self._center_hold_since = None
        self._forward_until = None
        self._pause_until = None
        self._front_since = None
        self._above_since = None
        self._charge_elapsed = 0.0
        self._lifting = False
        self._lift_trial_elapsed = 0.0
        self._lift_forced = False
        self._lift_elapsed = 0.0
        self._n.reset_pids()
        self._n.publish_state(new_state.name)

    def _pinger_above_held(self, now: float) -> bool:
        """ピンガーが「ロボットより上」(伏角 <= -pinger_above_pitch_deg)の判定が
        pinger_above_hold_s 連続で成立し続けているかを返す。

        PINGER_APPROACH(→即ミッション完了)とCHARGE(→突進中断して離脱)で使う。
        条件が一度でも外れたらカウントし直すので、外れ値では成立しない。
        """
        n = self._n
        dep = n.pinger_depression_deg
        if dep is not None and dep <= -n.p("pinger_above_pitch_deg"):
            if self._above_since is None:
                self._above_since = now
        else:
            self._above_since = None
        return (
            self._above_since is not None
            and now - self._above_since >= n.p("pinger_above_hold_s")
        )

    def _start_pause(self, now: float, next_state: State) -> None:
        """1動作の終わりに小休止(PAUSE)を挟んでから next_state へ進む。"""
        self._pause_next = next_state
        self._transition(now, State.PAUSE)

    def _enter_approach(self, now: float) -> None:
        """step3へ(再)突入する準備: 観測(collect)からやり直す。"""
        self._reset_chase()
        self._transition(now, State.PINGER_APPROACH)

    def _in_state_for(self, now: float) -> float:
        return now - self._state_entered

    def _reset_chase(self) -> None:
        """step3のサブモードを「観測から」やり直す状態に戻す。"""
        self._chase_mode = "collect"
        self._bearing_samples = []

    def _chase_wrench(self, now: float, dt: float) -> tuple[float, float]:
        """step3の追尾処理。(前進力force.x, ヨートルクtorque.z)を返す。

        サブモードの流れ:
          collect: 停止してピンガーを pinger_avg_samples 回観測し、外れ値を除いた
                   平均方位を得る。既に正面(±yaw_tolerance_deg)なら即前進、
                   ズレていればヨー目標にセットして turn
          turn   : 目標へ旋回。±yaw_tolerance_deg に0.5秒収まったらその角度で停止
                   → recheck
          recheck: 停止したまま再度観測(collectと同じ判定)。正面なら前進、
                   まだズレていれば turn へ戻る
          forward: approach_forward_s 秒前進。ヨーは制御しない(トルク0)。
                   開始時にヨーPIDをリセットしI項を引き継がない
          rest   : action_pause_s 秒小休止(その場の機首を保持) → collect へ
        ※ ピンガーは約1Hz更新。観測はnode側のメディアンフィルタ(単発の飛び除去)を
          通った値なので、pinger_avg_samples は2〜3回で十分。
        """
        n = self._n
        mode = self._chase_mode
        if mode == "forward":
            if now < self._forward_until:
                return n.p("approach_forward_force_n"), 0.0  # 前進中はヨーを触らない
            # 前進終了: 流れた先の機首をそのまま保持して小休止
            self._forward_until = None
            self.yaw_target_deg = n.yaw_deg
            n.reset_yaw_pid()
            self._pause_until = now + n.p("action_pause_s")
            self._chase_mode = mode = "rest"
        if mode == "rest":
            if now < self._pause_until:
                return 0.0, self._yaw_torque(dt)
            self._pause_until = None
            self._bearing_samples = []
            self._chase_mode = mode = "collect"
        if mode in ("collect", "recheck"):
            if len(self._bearing_samples) < n.p("pinger_avg_samples", int):
                return 0.0, self._yaw_torque(dt)  # 停止したまま観測を待つ
            target = robust_circular_mean_deg(
                self._bearing_samples, n.p("pinger_avg_outlier_deg")
            )
            self._bearing_samples = []
            if abs(wrap_deg(target - n.yaw_deg)) <= n.p("yaw_tolerance_deg"):
                # 停止観測で正面を確認できた → ヨーPIDをリセットして前進開始
                # (collectで既に正面なら旋回・再確認は省略して時間を稼ぐ)
                n.reset_yaw_pid()
                if self.state is State.RISE_ALIGN:
                    # step4': ヨー合わせ完了。前進はせず次の段階へ
                    if n.p("enable_vision", bool):
                        self._start_pause(now, State.VISION_ALIGN)
                    else:
                        n.get_logger().info("aligned after rise: charging")
                        self._start_pause(now, State.CHARGE)
                    return 0.0, 0.0
                self._forward_until = now + n.p("approach_forward_s")
                self._chase_mode = "forward"
                return n.p("approach_forward_force_n"), 0.0
            self.yaw_target_deg = target
            n.reset_yaw_pid()
            self._settle_since = None
            self._chase_mode = "turn"
            return 0.0, self._yaw_torque(dt)
        # mode == "turn": 正面が0.5秒続いたら停止して再観測へ
        if abs(n.yaw_error_deg(self.yaw_target_deg)) <= n.p("yaw_tolerance_deg"):
            if self._settle_since is None:
                self._settle_since = now
            elif now - self._settle_since >= 0.5:
                n.reset_yaw_pid()
                self._bearing_samples = []
                self._settle_since = None
                self._chase_mode = "recheck"
        else:
            self._settle_since = None
        return 0.0, self._yaw_torque(dt)

    def _settled(self, now: float, check_yaw: bool) -> bool:
        """深度(と必要ならヨー)が許容内に settle_duration_s の間収まり続けたらTrue。"""
        n = self._n
        ok = (
            self.depth_target_m is not None
            and n.depth_m is not None
            and abs(self.depth_target_m - n.depth_m) <= n.p("depth_tolerance_m")
        )
        if check_yaw:
            ok = ok and abs(n.yaw_error_deg(self.yaw_target_deg)) <= n.p("yaw_tolerance_deg")
        if not ok:
            self._settle_since = None
            return False
        if self._settle_since is None:
            self._settle_since = now
            return False
        return now - self._settle_since >= n.p("settle_duration_s")

    def _depth_force(self, dt: float) -> float:
        return self._n.depth_force(dt, self.depth_target_m)

    def _yaw_torque(self, dt: float) -> float:
        return self._n.yaw_torque(dt, self.yaw_target_deg)
