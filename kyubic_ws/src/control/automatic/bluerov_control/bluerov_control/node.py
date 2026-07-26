"""BlueROVミッション制御ノード。blueRovControl/main.py の10Hz whileループを
create_timer()に置き換えたもの。

トピック/サービス名は全て相対名で作成し、実体(mavlink_driver, localization,
emergency_surfacing)への配線は launch/bluerov_control.launch.py 側の remapping で行う
(このリポジトリの既存launchファイル群と同じ流儀。node.py自体はどの機体か・
どの名前空間かを知らない)。
"""

import numpy as np
import rclpy
from bluerov_control_msgs.msg import BuoyDetection, PingerDirection
from driver_msgs.msg import VehicleState
from geometry_msgs.msg import WrenchStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from localization_msgs.msg import Odometry
from planner_msgs.msg import WrenchPlan
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from . import control, flow

HEARTBEAT_PERIOD_SEC = 0.2  # mavlink_driverのcontrol_heartbeat_timeout_s(既定1.0s)より十分短い


class BlueRovControlNode(Node):
    def __init__(self) -> None:
        super().__init__("bluerov_control_node")

        mission_params = self._declare_mission_params()
        gains = self._declare_pid_gains()
        position_tolerance_m = self.declare_parameter("position_tolerance_m", 0.2).value
        depth_tolerance_m = self.declare_parameter("depth_tolerance_m", 0.1).value
        surface_z_threshold_m = self.declare_parameter("surface_z_threshold_m", 0.1).value
        self._odom_timeout_s = self.declare_parameter("odom_timeout_s", 1.0).value
        main_loop_hz = self.declare_parameter("main_loop_hz", 10.0).value
        self._emergency_change_state_service = self.declare_parameter(
            "emergency_change_state_service", "/emergency/emergency_surfacing/change_state"
        ).value
        self.declare_parameter("debug.image_processing_available", True)
        self.declare_parameter("debug.mic_data_from_above", False)

        self._control_loop = control.ControlLoop(
            gains=gains,
            position_tolerance=position_tolerance_m,
            depth_tolerance=depth_tolerance_m,
            surface_z_threshold=surface_z_threshold_m,
        )
        self._ctx = flow.MissionContext(params=mission_params, tick_fn=self._tick_fn)
        self._state = flow.State.INIT

        self._latest_odom: Odometry | None = None
        self._last_odom_received_time = None  # rclpy.time.Time。_odom_callbackが受信毎に更新
        self._latest_vehicle_state: VehicleState | None = None
        self._mission_start = None
        self._last_control_tick_time = None
        self._armed_requested = False
        self._mission_finished_logged = False

        self._robot_force_pub = self.create_publisher(WrenchStamped, "robot_force", 10)
        self._heartbeat_pub = self.create_publisher(Bool, "heartbeat", 10)

        self.create_subscription(Odometry, "odom", self._odom_callback, 10)
        self.create_subscription(VehicleState, "vehicle_state", self._vehicle_state_callback, 10)
        self.create_subscription(
            PingerDirection, "pinger_direction", self._pinger_direction_callback, 10
        )
        self.create_subscription(BuoyDetection, "buoy_detection", self._buoy_detection_callback, 10)
        self.create_subscription(WrenchPlan, "wrench_plan", self._wrench_plan_callback, 10)

        self._set_armed_client = self.create_client(SetBool, "set_armed")
        self._emergency_change_state_client = self.create_client(
            ChangeState, self._emergency_change_state_service
        )

        self.create_timer(1.0 / max(float(main_loop_hz), 1.0), self._tick)
        self.create_timer(HEARTBEAT_PERIOD_SEC, self._publish_heartbeat)

        self.get_logger().info("bluerov_control_node を起動しました。初期状態: INIT")

    # ---- パラメータ宣言 ----

    def _declare_mission_params(self) -> flow.MissionParams:
        return flow.MissionParams(
            init_wait_sec=self.declare_parameter("init_wait_sec", 180.0).value,
            search_timeout_sec=self.declare_parameter("search_timeout_sec", 420.0).value,
            mic_confirm_timeout_sec=self.declare_parameter("mic_confirm_timeout_sec", 45.0).value,
            hydrophone_pitch_threshold_deg=self.declare_parameter(
                "hydrophone_pitch_threshold_deg", 30.0
            ).value,
            hydrophone_score_threshold=self.declare_parameter(
                "hydrophone_score_threshold", 0.0
            ).value,
            yolo_confidence_threshold=self.declare_parameter(
                "yolo_confidence_threshold", 0.5
            ).value,
            yolo_stale_timeout_sec=self.declare_parameter("yolo_stale_timeout_sec", 1.0).value,
            descend_depth_m=self.declare_parameter("descend_depth_m", 7.0).value,
            dive_offset_m=self.declare_parameter("dive_offset_m", 2.5).value,
            post_check_rise_offset_m=self.declare_parameter("post_check_rise_offset_m", 0.5).value,
            surface_rise_offset_m=self.declare_parameter("surface_rise_offset_m", 4.0).value,
        )

    def _declare_pid(
        self,
        name: str,
        kp: float,
        ki: float,
        kd: float,
        output_limits: tuple,
        windup_limit: float,
        angle_error_deg: bool,
    ) -> dict:
        kp_v = self.declare_parameter(f"pid.{name}.kp", kp).value
        ki_v = self.declare_parameter(f"pid.{name}.ki", ki).value
        kd_v = self.declare_parameter(f"pid.{name}.kd", kd).value
        limits_v = self.declare_parameter(f"pid.{name}.output_limits", list(output_limits)).value
        windup_v = self.declare_parameter(f"pid.{name}.windup_limit", windup_limit).value
        angle_v = self.declare_parameter(f"pid.{name}.angle_error_deg", angle_error_deg).value
        return {
            "kp": kp_v,
            "ki": ki_v,
            "kd": kd_v,
            "output_limits": tuple(limits_v),
            "windup_limit": windup_v,
            "angle_error_deg": angle_v,
        }

    def _declare_velocity_pid(
        self,
        name: str,
        kp: float,
        ki: float,
        kd: float,
        output_limits: tuple,
        kf: float = 0.0,
        offset: float = 0.0,
        angle_error_deg: bool = False,
    ) -> dict:
        """内側(速度)ループ用。`VelocityPID`(pid_controller::VelocityPIDの移植)のパラメータを宣言する。

        `_declare_pid`のwindup_limitに代わり、D項ローパスフィルタ係数kfと
        フィードフォワードoffsetを宣言する(速度形PIDは出力を毎周期クランプするため
        専用のwindup_limitは不要)。
        """
        kp_v = self.declare_parameter(f"pid.{name}.kp", kp).value
        ki_v = self.declare_parameter(f"pid.{name}.ki", ki).value
        kd_v = self.declare_parameter(f"pid.{name}.kd", kd).value
        limits_v = self.declare_parameter(f"pid.{name}.output_limits", list(output_limits)).value
        kf_v = self.declare_parameter(f"pid.{name}.kf", kf).value
        offset_v = self.declare_parameter(f"pid.{name}.offset", offset).value
        angle_v = self.declare_parameter(f"pid.{name}.angle_error_deg", angle_error_deg).value
        return {
            "kp": kp_v,
            "ki": ki_v,
            "kd": kd_v,
            "output_limits": tuple(limits_v),
            "kf": kf_v,
            "offset": offset_v,
            "angle_error_deg": angle_v,
        }

    def _declare_pid_gains(self) -> dict:
        # 各ゲインの初期値は blueRovControl/params.py と同じ。ただし出力先が
        # MAVLink MANUAL_CONTROLスケールからWrenchStamped/axis.limitスケールに変わったため、
        # 実機でのリチューニングが前提(README参照)。
        return {
            "surge_outer": self._declare_pid("surge_outer", 0.15, 0.0, 0.0, (0.0, 0.3), 0.6, False),
            "heave_outer": self._declare_pid("heave_outer", 0.3, 0.0, 0.0, (-0.3, 0.3), 0.6, False),
            "yaw_outer": self._declare_pid("yaw_outer", 0.5, 0.0, 0.0, (-20.0, 20.0), 40.0, True),
            "surge_inner": self._declare_velocity_pid(
                "surge_inner", 1000.0, 0.0, 0.0, (0.0, 400.0)
            ),
            "heave_inner": self._declare_velocity_pid(
                "heave_inner", 800.0, 0.0, 0.0, (-300.0, 300.0)
            ),
            "yaw_inner": self._declare_velocity_pid(
                "yaw_inner", 20.0, 0.0, 0.0, (-500.0, 500.0)
            ),
        }

    # ---- Subscriptionコールバック(最新値を保持するだけ) ----

    def _odom_callback(self, msg: Odometry) -> None:
        self._latest_odom = msg
        self._last_odom_received_time = self.get_clock().now()

    def _vehicle_state_callback(self, msg: VehicleState) -> None:
        self._latest_vehicle_state = msg

    def _pinger_direction_callback(self, msg: PingerDirection) -> None:
        self._ctx.pinger_direction = msg

    def _buoy_detection_callback(self, msg: BuoyDetection) -> None:
        self._ctx.buoy_detection = msg

    def _wrench_plan_callback(self, msg: WrenchPlan) -> None:
        self._ctx.wrench_plan = msg

    # ---- メインループ ----

    def _tick(self) -> None:
        now = self.get_clock().now()
        if self._mission_start is None:
            self._mission_start = now
        self._ctx.elapsed_time = (now - self._mission_start).nanoseconds * 1e-9
        self._ctx.now_sec = now.nanoseconds * 1e-9
        self._ctx.image_processing_available = self.get_parameter(
            "debug.image_processing_available"
        ).value
        self._ctx.mic_data_from_above = self.get_parameter("debug.mic_data_from_above").value

        odom = self._latest_odom
        if odom is not None:
            self._ctx.position = np.array(
                [odom.pose.position.x, odom.pose.position.y, odom.pose.position.z_depth]
            )
            self._ctx.orientation_deg = (
                odom.pose.orientation.x,
                odom.pose.orientation.y,
                odom.pose.orientation.z,
            )

        if self._state == flow.State.INIT and not self._armed_requested:
            self._request_arm()
            self._armed_requested = True

        # デフォルトは中立(推力ゼロ)。tick_fnが呼ばれるStateではこの後上書きされる。
        self._publish_wrench(0.0, 0.0, 0.0)

        previous_state = self._state
        new_state = flow.mission_tick(self._ctx, self._state)
        if new_state != previous_state:
            self.get_logger().info(
                f"{previous_state.name} -> {new_state.name} (t={self._ctx.elapsed_time:.1f}s)"
            )
            if new_state == flow.State.EMERGENCY:
                self._enter_emergency()
        self._state = new_state

        if (
            self._state in (flow.State.DONE, flow.State.EMERGENCY)
            and not self._mission_finished_logged
        ):
            self.get_logger().info(f"ミッション終了: {self._state.name}")
            self._publish_wrench(0.0, 0.0, 0.0)
            self._mission_finished_logged = True

    def _tick_fn(self, dx: float, dy: float, dz: float) -> control.ControlResult:
        """flow.py の各Stateハンドラから呼ばれる、制御1周期分の実行。"""
        now = self.get_clock().now()
        dt = (
            0.0
            if self._last_control_tick_time is None
            else (now - self._last_control_tick_time).nanoseconds * 1e-9
        )
        self._last_control_tick_time = now

        odom = self._latest_odom
        if odom is None:
            # /localization/odom をまだ一度も受信していない。安全側に倒し、中立のまま待機する。
            self._publish_wrench(0.0, 0.0, 0.0)
            return control.ControlResult(emergency=False, reached=False)

        position_z = odom.pose.position.z_depth
        roll_deg = odom.pose.orientation.x
        pitch_deg = odom.pose.orientation.y
        yaw_deg = odom.pose.orientation.z
        vx_world = odom.twist.linear.x
        vy_world = odom.twist.linear.y
        vz_world = odom.twist.linear.z_depth
        yaw_rate_deg = odom.twist.angular.z

        surge_velocity_body = control.world_velocity_to_body_surge(
            vx_world, vy_world, vz_world, roll_deg, pitch_deg, yaw_deg
        )

        # masudanote.md Day4「位置センサー死亡監視」: odomが一定時間更新されていなければ、
        # position_z等が古い値でも構わずEMERGENCYへ倒す(ControlLoop.is_emergency参照)。
        odom_age_sec = (now - self._last_odom_received_time).nanoseconds * 1e-9
        odom_stale = odom_age_sec > self._odom_timeout_s

        result = self._control_loop.tick(
            dx,
            dy,
            dz,
            position_z=position_z,
            yaw_deg=yaw_deg,
            surge_velocity_body=surge_velocity_body,
            heave_velocity_world=vz_world,
            yaw_rate_deg=yaw_rate_deg,
            dt=dt,
            odom_stale=odom_stale,
        )

        if result.emergency or result.reached:
            self._publish_wrench(0.0, 0.0, 0.0)
        else:
            self._publish_wrench(result.force_x, result.force_z, result.torque_z)
        return result

    # ---- Publish ----

    def _publish_wrench(self, force_x: float, force_z: float, torque_z: float) -> None:
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.wrench.force.x = float(force_x)
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = float(force_z)
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = float(torque_z)
        self._robot_force_pub.publish(msg)

    def _publish_heartbeat(self) -> None:
        msg = Bool()
        msg.data = True
        self._heartbeat_pub.publish(msg)

    # ---- Arm / 緊急停止(サービス呼び出し) ----

    def _request_arm(self) -> None:
        if not self._set_armed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(
                "set_armed サービスが見つかりません。武装は手動で行ってください"
            )
            return
        request = SetBool.Request()
        request.data = True
        future = self._set_armed_client.call_async(request)
        future.add_done_callback(self._on_arm_response)

    def _on_arm_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 - サービス呼び出し失敗は継続可能な扱いにする
            self.get_logger().error(f"武装リクエストに失敗しました: {error}")
            return
        if response.success:
            self.get_logger().info("武装に成功しました")
        else:
            self.get_logger().warning(f"武装に失敗しました: {response.message}")

    def _enter_emergency(self) -> None:
        """emergency_surfacing(lifecycleノード)を configure -> activate し、
        一定の上向き推力での緊急浮上を開始させる。"""
        self._call_lifecycle_transition(Transition.TRANSITION_CONFIGURE)
        self._call_lifecycle_transition(Transition.TRANSITION_ACTIVATE)

    def _call_lifecycle_transition(self, transition_id: int) -> None:
        if not self._emergency_change_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(
                f"{self._emergency_change_state_service} が見つかりません(emergency_surfacingが"
                " 起動しているか確認してください)"
            )
            return
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self._emergency_change_state_client.call_async(request)
        future.add_done_callback(lambda f: self._log_lifecycle_result(transition_id, f))

    def _log_lifecycle_result(self, transition_id: int, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(
                f"emergency_surfacing への遷移要求(id={transition_id})に失敗: {error}"
            )
            return
        if not response.success:
            self.get_logger().warning(
                f"emergency_surfacing への遷移要求(id={transition_id})が拒否されました"
            )

    def destroy_node(self) -> bool:
        self._publish_wrench(0.0, 0.0, 0.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlueRovControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
