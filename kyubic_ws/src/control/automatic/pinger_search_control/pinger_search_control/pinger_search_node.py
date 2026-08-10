"""BlueROVピンガー探査ミッションノードのROS I/O層。

ミッションの進行(step1〜9のステート遷移)は mission_fsm.MissionFsm に実装してあり、
このノードは以下だけを担う:

  - センサ購読と鮮度管理: imu(yaw)・depth・pinger・YOLO・vehicle_state・start trigger
  - ピンガー外れ値フィルタ(pinger_filter.PingerFilter)の適用
  - 配信: robot_force / heartbeat(常時true) / 画処理enable / 現在ステート(~/state)
  - set_armed サービスの呼び出し(ARM/DISARM)
  - 深度・ヨーPID: FSMから目標値を渡されて推力(force.z / torque.z)を返す

座標・符号の規約(ハイドロフォン班確認済み):
  - ピンガー yaw  : ロボット正面=0deg、右=正(+180まで)、左=負(-180まで)
  - ピンガー pitch: 上=正、下=負 (内部では伏角 = -pitch、正=真下方向として扱う)
  - force.z 正=潜航方向(mavlink_driverのheave軸)、torque.z 正=右旋回
    (実機で逆なら axis.heave.invert / yaw_torque_sign で反転する)

DVL・localizationは使用しない。
"""

import math
import time

import rclpy
from buoy_interfaces.msg import BuoyRelativePosition
from driver_msgs.msg import IMU, LED, BoolStamped, Depth, VehicleState
from geometry_msgs.msg import WrenchStamped
from planner_msgs.msg import PingerDirection
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from .mission_fsm import MissionFsm
from .pid import PID
from .pinger_filter import PingerFilter, wrap_deg

CONTROL_PERIOD_SEC = 0.1
HEARTBEAT_PERIOD_SEC = 0.2  # mavlink_driverのcontrol_heartbeat_timeout_s(既定1.0s)より十分短い
ARM_RETRY_PERIOD_SEC = 3.0


class PingerSearchNode(Node):
    def __init__(self) -> None:
        super().__init__("pinger_search_node")

        self._declare_parameters()

        # --- センサ購読値(FSMから参照される) ---
        self.yaw_deg: float | None = None  # IMU orient.z [deg]
        self.depth_m: float | None = None  # 正=深い [m]
        self.vehicle_armed = False
        self.start_triggered = False
        self.buoy: BuoyRelativePosition | None = None
        self.buoy_time = 0.0
        self._yaw_time = 0.0
        self._depth_time = 0.0

        # --- ピンガー(フィルタ後、FSMから参照される) ---
        self._pinger_filter = PingerFilter(
            window=self.p("pinger_filter_window", int),
            score_max=self.p("pinger_score_max"),
        )
        self.pinger_yaw_deg: float | None = None  # ロボット相対 [deg] 正=右
        self.pinger_depression_deg: float | None = None  # 伏角 [deg] 正=真下方向
        self.pinger_seq = 0  # 採用サンプル数(FSMの更新待ちに使う)

        # --- FSM / PID ---
        self._fsm = MissionFsm(self)
        self._last_tick: float | None = None
        self._depth_pid = PID(
            kp=self.p("depth_kp"),
            ki=self.p("depth_ki"),
            kd=self.p("depth_kd"),
            output_limit=self.p("depth_force_limit_n"),
        )
        self._yaw_pid = PID(
            kp=self.p("yaw_kp"),
            ki=self.p("yaw_ki"),
            kd=self.p("yaw_kd"),
            output_limit=self.p("yaw_torque_limit_nm"),
        )

        # --- ARMサービス ---
        self._arm_client = self.create_client(SetBool, "set_armed")
        self._arm_future = None
        self._arm_request_time = -math.inf

        # --- pub/sub/timer ---
        self.create_subscription(BoolStamped, "mission_start_trigger", self._start_callback, 10)
        self.create_subscription(IMU, "imu", self._imu_callback, 10)
        self.create_subscription(Depth, "depth", self._depth_callback, 10)
        self.create_subscription(PingerDirection, "pinger_direction", self._pinger_callback, 10)
        self.create_subscription(BuoyRelativePosition, "buoy_position", self._buoy_callback, 10)
        self.create_subscription(VehicleState, "vehicle_state", self._vehicle_state_callback, 10)

        self._wrench_pub = self.create_publisher(WrenchStamped, "robot_force", 10)
        self._heartbeat_pub = self.create_publisher(Bool, "heartbeat", 10)
        self._image_enable_pub = self.create_publisher(Bool, "image_processing_enable", 10)
        self._led_pub = self.create_publisher(LED, "led", 10)
        self._state_pub = self.create_publisher(String, "~/state", 10)

        self.create_timer(CONTROL_PERIOD_SEC, self._control_tick)
        self.create_timer(HEARTBEAT_PERIOD_SEC, self._publish_heartbeat)

        self.get_logger().info("pinger_search_node started: waiting for mission_start_trigger")

    # ------------------------------------------------------------ パラメータ

    def _declare_parameters(self) -> None:
        p = self.declare_parameter
        # ミッション定数
        p("start_delay_s", 30.0)  # トリガ後、Yaw1記憶・ミッション開始までの待機時間
        p("mission_timeout_s", 540.0)  # step9: 開始(待機明け)から9分で強制浮上
        p("cruise_depth_m", 4.0)  # step2: 探査時の保持深度
        p("finish_depth_m", 1.0)  # step8/9: 終了時の浮上深度
        p("max_depth_m", 8.0)  # step4で許す最大深度(安全上限)
        p("depth_step_m", 0.5)  # step4の1ステップ降下量
        # 推力・時間
        p("dash_force_n", 30.0)  # step1の前進力
        p("dash_duration_s", 3.0)
        p("approach_forward_force_n", 20.0)  # step3の前進力
        p("approach_forward_s", 2.0)
        p("charge_force_n", 30.0)  # step6の突進力
        p("charge_duration_s", 3.0)
        p("post_charge_lift_m", 0.2)  # step6終了後: ブイを引っ張って外すため浮上する量
        p("post_charge_lift_trial_s", 4.0)  # まず通常の深度PIDで浮上を試みる時間。
        # これを超えても到達しなければ「ブイに引っかかった」と判定し最大推力へ切り替える
        p("post_charge_lift_duration_s", 3.0)  # 最大推力へ切り替えた後、前進しながら行う時間
        p("post_charge_lift_force_n", 20.0)  # 上記の浮上力(深度PIDではなく固定の最大推力)
        p("retreat_force_n", 20.0)  # step7の後進力(正の値。後進時に-xで印加)
        p("retreat_duration_s", 5.0)
        p("action_pause_s", 2.0)  # 1動作(ダッシュ/前進/突進/後進等)後の小休止時間
        # 到達判定
        p("depth_tolerance_m", 0.2)
        p("yaw_tolerance_deg", 8.0)
        p("settle_duration_s", 1.5)  # 深度/ヨー到達とみなすまでの連続滞在時間
        # ステートタイムアウト(条件待ちのハング防止。超過で次のステートへ強制遷移)
        p("state_timeout_s", 120.0)  # 条件待ちステート共通の上限滞在時間
        p("approach_timeout_s", 300.0)  # PINGER_APPROACHのみ(移動距離があるので長め)
        # ピンガー判定
        p("pinger_filter_window", 3)  # メディアンフィルタ窓(奇数推奨)
        p("pinger_score_max", 100.0)  # これを超えるscore(誤差大)のサンプルは棄却
        p("pinger_avg_samples", 2)  # step3: 旋回目標を決める観測回数(外れ値除去平均)。
        # ピンガーは約1Hz+メディアン済みなので2〜3で十分(多いと停止時間が延びる)
        p("pinger_avg_outlier_deg", 20.0)  # 平均時、メディアンからこれ超のサンプルを棄却
        p("pinger_yaw_sign", 1.0)  # yaw正=右(確認済み)。逆機体が出たら-1.0
        p("pinger_depression_sign", -1.0)  # 伏角=sign*pitch。pitch負=下(確認済み)なので-1.0
        p("pinger_below_pitch_deg", 60.0)  # 伏角がこれ以上で「ほぼ真下」(step4開始)
        p("pinger_front_pitch_deg", 15.0)  # |伏角|がこの範囲内で「正面」(step4終了)
        p("pinger_above_pitch_deg", 10.0)  # 伏角が-これより上で「ロボットより上」(step8)
        p("pinger_above_hold_s", 10.0)  # 「上」判定がこの秒数連続したら: 探査中→終了、突進中→離脱
        p("pinger_front_confirm_s", 3.0)  # step4: 正面条件がこの秒数連続成立で「確実」とみなす
        p("charge_rise_m", 0.2)  # step4': 突進前に浮上する量(ピンガーが目標物の下側に付くため)
        p("surface_check_duration_s", 5.0)  # step8でピンガーを観測する最低時間
        # 画像処理(YOLO)中心合わせ
        p("enable_vision", True)  # falseならstep5-6をスキップし、正面確認後そのままCHARGE(数秒前進)
        p("center_tolerance", 0.15)  # 正規化偏差の中央判定閾値
        p("center_hold_s", 1.5)  # 中央に居続ける必要時間
        p("vision_yaw_rate_deg_s", 20.0)  # 水平偏差1.0あたりのyaw目標変化速度
        p("buoy_timeout_s", 1.0)  # 検出がこの時間途絶えたらピンガーアプローチに戻る
        p("light_pwm", 1500)  # 画処理中のライト明るさ(PWM 1100=消灯〜1900=最大)
        # 深度PID(出力=force.z [N]、正=潜航)
        p("depth_kp", 15.0)
        p("depth_ki", 0.5)
        p("depth_kd", 8.0)
        p("depth_force_limit_n", 15.0)
        # ヨーPID(入力=角度誤差[deg]、出力=torque.z [Nm]、正=右旋回)
        p("yaw_kp", 0.15)
        p("yaw_ki", 0.0)
        p("yaw_kd", 0.08)
        p("yaw_torque_limit_nm", 8.0)
        p("yaw_torque_sign", 1.0)  # torque.z正=右旋回でない場合は-1.0
        # 監視
        p("sensor_timeout_s", 1.0)  # imu/depthがこの時間途絶えたら中立出力

    def p(self, name: str, cast=float):
        """パラメータ取得(FSMからも使う)。"""
        return cast(self.get_parameter(name).value)

    # ------------------------------------------------------------- callbacks

    def _start_callback(self, msg: BoolStamped) -> None:
        if msg.data:
            self.start_triggered = True

    def _imu_callback(self, msg: IMU) -> None:
        if math.isfinite(msg.orient.z):
            self.yaw_deg = wrap_deg(msg.orient.z)
            self._yaw_time = time.monotonic()

    def _depth_callback(self, msg: Depth) -> None:
        if math.isfinite(msg.depth):
            self.depth_m = float(msg.depth)
            self._depth_time = time.monotonic()

    def _vehicle_state_callback(self, msg: VehicleState) -> None:
        self.vehicle_armed = msg.armed

    def _buoy_callback(self, msg: BuoyRelativePosition) -> None:
        self.buoy = msg
        self.buoy_time = time.monotonic()

    def _pinger_callback(self, msg: PingerDirection) -> None:
        """外れ値フィルタを通し、採用サンプルならFSMへ通知する。"""
        filtered = self._pinger_filter.update(msg.yaw, msg.pitch, msg.score)
        if filtered is None:
            self.get_logger().warning(
                f"pinger sample rejected (score={msg.score:.1f})",
                throttle_duration_sec=5.0,
            )
            return
        yaw, pitch = filtered
        self.pinger_yaw_deg = self.p("pinger_yaw_sign") * yaw
        self.pinger_depression_deg = self.p("pinger_depression_sign") * pitch
        self.pinger_seq += 1
        self._fsm.on_pinger_accepted()

    # ------------------------------------------------------------- main loop

    def _control_tick(self) -> None:
        now = time.monotonic()
        dt = CONTROL_PERIOD_SEC if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        self._fsm.tick(now, dt)

    # ------------------------------------------------- FSMから使うヘルパ群

    def sensors_stale(self, now: float) -> bool:
        """imu/depthが未受信、またはsensor_timeout_sより古ければTrue。"""
        timeout = self.p("sensor_timeout_s")
        if self.yaw_deg is None or self.depth_m is None:
            return True
        return now - self._yaw_time > timeout or now - self._depth_time > timeout

    def yaw_error_deg(self, target_deg: float | None) -> float:
        if target_deg is None or self.yaw_deg is None:
            return 0.0
        return wrap_deg(target_deg - self.yaw_deg)

    def yaw_torque(self, dt: float, target_deg: float | None) -> float:
        if target_deg is None or self.yaw_deg is None:
            return 0.0
        return self.p("yaw_torque_sign") * self._yaw_pid.update(
            self.yaw_error_deg(target_deg), dt
        )

    def depth_force(self, dt: float, target_m: float | None) -> float:
        if target_m is None or self.depth_m is None:
            return 0.0
        # 深度は正=深い、force.z(heave)は正=潜航方向
        return self._depth_pid.update(target_m - self.depth_m, dt)

    def reset_pids(self) -> None:
        self._depth_pid.reset()
        self._yaw_pid.reset()

    def reset_yaw_pid(self) -> None:
        """ヨーPIDのみリセット(step3の旋回/前進切替でI項を引き継がないため)。"""
        self._yaw_pid.reset()

    def reset_pinger_filter(self) -> None:
        self._pinger_filter.reset()
        self.pinger_depression_deg = None

    def retry_arm(self, armed: bool, now: float) -> None:
        """ARM/DISARM要求。応答待ち中は何もしない。前回要求から一定時間後に再送する。"""
        if self._arm_future is not None and not self._arm_future.done():
            return
        if now - self._arm_request_time < ARM_RETRY_PERIOD_SEC:
            return
        if not self._arm_client.service_is_ready():
            self.get_logger().warning("set_armed service not ready", throttle_duration_sec=5.0)
            self._arm_request_time = now
            return
        request = SetBool.Request()
        request.data = armed
        self._arm_future = self._arm_client.call_async(request)
        self._arm_request_time = now
        self.get_logger().info(f"set_armed({armed}) requested")

    def poll_arm_response(self) -> bool | None:
        """set_armedの応答を取り出す。未着ならNone、着いていればsuccessの真偽。"""
        if self._arm_future is None or not self._arm_future.done():
            return None
        result = self._arm_future.result()
        self._arm_future = None
        if result is None or not result.success:
            self.get_logger().error(
                f"set_armed failed: {result.message if result is not None else 'no response'}"
            )
            return False
        return True

    # ---------------------------------------------------------------- 配信

    def publish_wrench(
        self, force_x: float, force_z: float, torque_z: float, force_y: float = 0.0
    ) -> None:
        """推力指令の配信。force.y(lateral)は正=右移動(mavlink_driverのlateral軸)。"""
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.wrench.force.x = float(force_x)
        msg.wrench.force.y = float(force_y)
        msg.wrench.force.z = float(force_z)
        msg.wrench.torque.z = float(torque_z)
        self._wrench_pub.publish(msg)

    def publish_image_enable(self, enable: bool) -> None:
        """画処理ON/OFFの配信。ライトも連動する(ON=light_pwm、OFF=1100で消灯)。"""
        msg = Bool()
        msg.data = enable
        self._image_enable_pub.publish(msg)
        led = LED()
        led.header.stamp = self.get_clock().now().to_msg()
        pwm = self.p("light_pwm", int) if enable else 1100
        led.left = pwm
        led.right = pwm
        self._led_pub.publish(led)

    def publish_state(self, name: str) -> None:
        msg = String()
        msg.data = name
        self._state_pub.publish(msg)

    def _publish_heartbeat(self) -> None:
        msg = Bool()
        msg.data = True
        self._heartbeat_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PingerSearchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
