import asyncio
import collections
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

from fastapi.responses import StreamingResponse
from nicegui import app, ui

# ==========================================
# Section 1: ROS 2 Imports & Mocking
# ==========================================
# ROS環境がない場所でもUI開発ができるように、インポートエラーをハンドリングします
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from cv_bridge import CvBridge
    import cv2
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import WrenchStamped
    from driver_msgs.msg import (
        BoolStamped,
        Depth,
        Gnss,
        IMU,
        Int32Stamped,
        LED,
        PowerState,
        VehicleState,
    )
    from blue_rov_msgs.msg import DVL75
    from localization_msgs.msg import Odometry

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("Warning: ROS 2 libraries not found. Running in UI-only mode.")

    class Node:
        def __init__(self, name):
            pass

        def declare_parameter(self, name, value):
            return type("obj", (), {"value": value})

        def create_subscription(self, *args, **kwargs):
            pass

        def create_publisher(self, *args, **kwargs):
            return type("pub", (), {"publish": lambda self, msg: None})()

        def create_client(self, *args, **kwargs):
            return None

        def create_timer(self, *args, **kwargs):
            return None

        def get_clock(self):
            return type("clock", (), {"now": lambda: type("t", (), {"to_msg": lambda: 0})})

        def destroy_node(self):
            pass

    Depth = Gnss = IMU = PowerState = VehicleState = DVL75 = object
    BoolStamped = Int32Stamped = LED = WrenchStamped = Bool = Odometry = object


# ==========================================
# Section 2: Styles & Constants
# ==========================================
class UIColors:
    """UIで使用するカラーパレット定数（CSS変数への参照）"""

    TEXT_MAIN = "var(--text-main)"
    TEXT_SUB = "var(--text-sub)"
    BORDER = "var(--border-color)"

    NORMAL = "var(--color-normal)"
    WARN = "var(--color-warn)"
    ERROR = "var(--color-error)"
    NEON_CYAN = "var(--color-cyan)"
    NEON_PURPLE = "var(--color-purple)"


CSS_STYLES = """
    @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Noto+Sans+JP:wght@300;500;700&display=swap');

    :root {
        --bg-base: #f1f5f9;
        --bg-gradient-1: rgba(16, 185, 129, 0.05);
        --bg-gradient-2: rgba(6, 182, 212, 0.05);
        --card-bg: rgba(255, 255, 255, 0.75);
        --border-color: rgba(148, 163, 184, 0.3);
        --text-main: #0f172a;
        --text-sub: #64748b;

        --color-normal: #059669;
        --color-warn: #eab308;
        --color-error: #dc2626;
        --color-cyan: #0891b2;
        --color-purple: #7c3aed;

        --shadow-color: rgba(0, 0, 0, 0.1);
        --radar-grid: rgba(51, 65, 85, 0.1);
    }

    body.body--dark {
        --bg-base: #0f172a;
        --bg-gradient-1: rgba(16, 185, 129, 0.08);
        --bg-gradient-2: rgba(6, 182, 212, 0.08);
        --card-bg: rgba(30, 41, 59, 0.6);
        --border-color: rgba(148, 163, 184, 0.1);
        --text-main: #f8fafc;
        --text-sub: #94a3b8;

        --color-normal: #10b981;
        --color-warn: #facc15;
        --color-error: #ff3333;
        --color-cyan: #06b6d4;
        --color-purple: #8b5cf6;

        --shadow-color: rgba(0, 0, 0, 0.3);
        --radar-grid: rgba(148, 163, 184, 0.1);
    }

    body {
        font-family: "Noto Sans JP", sans-serif;
        background-color: var(--bg-base);
        color: var(--text-main);
        background-image:
            radial-gradient(circle at 15% 50%, var(--bg-gradient-1), transparent 25%),
            radial-gradient(circle at 85% 30%, var(--bg-gradient-2), transparent 25%);
        transition: background-color 0.3s ease, color 0.3s ease;
    }

    .stat-value { font-family: 'Share Tech Mono', monospace; letter-spacing: 0.05em; }

    .cyber-card {
        background-color: var(--card-bg);
        backdrop-filter: blur(12px);
        -webkit-backdrop-filter: blur(12px);
        border: 1px solid var(--border-color);
        box-shadow: 0 4px 6px -1px var(--shadow-color);
        border-radius: 1rem;
        transition: all 0.3s ease;
    }

    .status-normal { border-color: var(--color-normal); box-shadow: 0 0 15px var(--color-normal); }
    .status-warn { border-color: var(--color-warn); box-shadow: 0 0 20px var(--color-warn); background-color: rgba(250, 204, 21, 0.15); }
    .status-error { border-color: var(--color-error); box-shadow: 0 0 30px var(--color-error); background-color: rgba(255, 51, 51, 0.2); }
    .status-timeout {
        border-color: var(--color-error);
        box-shadow: 0 0 30px var(--color-error);
        background-color: rgba(255, 51, 51, 0.2);
        animation: pulse-red 2s infinite;
    }

    @keyframes pulse-red {
        0% { box-shadow: 0 0 10px var(--color-error); opacity: 0.8; }
        50% { box-shadow: 0 0 40px 15px var(--color-error); opacity: 1; }
        100% { box-shadow: 0 0 10px var(--color-error); opacity: 0.8; }
    }

    .global-alert {
        box-shadow: inset 0 0 50px var(--color-error);
        animation: alert-blink 0.4s infinite alternate;
    }
    @keyframes alert-blink {
        from { box-shadow: inset 0 0 20px var(--color-error); }
        to { box-shadow: inset 0 0 100px var(--color-error); }
    }

    .radar-grid {
        background-image:
            linear-gradient(var(--radar-grid) 1px, transparent 1px),
            linear-gradient(90deg, var(--radar-grid) 1px, transparent 1px);
        background-size: 20px 20px;
        border-radius: 50%;
        border: 1px solid var(--border-color);
    }
"""


# ==========================================
# Section 3: Data Models
# ==========================================
@dataclass
class SystemConfig:
    timeout_depth: float = 1.0
    timeout_imu: float = 0.5
    timeout_power: float = 3.0
    timeout_gnss: float = 3.0
    timeout_vehicle_state: float = 1.0
    timeout_dvl: float = 1.0
    timeout_camera: float = 2.0
    volt_warn_low: float = 13.0
    volt_warn_critical: float = 11.5


@dataclass
class SensorData:
    last_update: float = 0.0
    status_id: int = 0

    def is_timeout(self, timeout_sec: float) -> bool:
        return (time.time() - self.last_update) > timeout_sec


@dataclass
class DepthData(SensorData):
    depth: float = 0.0
    temp: float = 0.0


@dataclass
class ImuData(SensorData):
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class PowerData(SensorData):
    voltage: float = 0.0
    current: float = 0.0
    power: float = 0.0


@dataclass
class GnssData(SensorData):
    fix_ok: bool = False
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0


@dataclass
class VehicleStateData(SensorData):
    connected: bool = False
    armed: bool = False
    custom_mode: int = 0
    system_status: int = 0


@dataclass
class DvlData(SensorData):
    velocity_north: float = 0.0
    velocity_east: float = 0.0
    velocity_up: float = 0.0
    altitude: float = 0.0
    confidence: int = 0
    bottom_lock: bool = False
    speed_norm: float = 0.0


@dataclass
class CameraData:
    last_update: float = 0.0
    jpg_bytes: bytes = b""

    def is_timeout(self, timeout_sec: float) -> bool:
        return (time.time() - self.last_update) > timeout_sec


@dataclass
class TrajectoryData(SensorData):
    x: float = 0.0
    y: float = 0.0
    depth_m: float = 0.0
    yaw: float = 0.0
    points: collections.deque = field(default_factory=lambda: collections.deque(maxlen=500))


@dataclass
class MissionStartData:
    last_update: float = 0.0
    triggered: bool = False
    triggered_at: float = 0.0


@dataclass
class LedControlState:
    """実機にはライトが1基のみ搭載(mavlink_driver設定でled_left_channel=0)。"""

    pwm: int = 1100


@dataclass
class ThrusterTestState:
    test_force_n: float = 15.0
    test_torque_nm: float = 2.0
    pulse_duration_s: float = 1.0
    manual_control_enabled: bool = False


class RobotState:
    """アプリケーション全体の状態を保持するクラス"""

    def __init__(self):
        self.config = SystemConfig()
        self.depth = DepthData()
        self.imu = ImuData()
        self.power = PowerData()
        self.gnss = GnssData()
        self.vehicle = VehicleStateData()
        self.dvl = DvlData()
        self.camera = CameraData()
        self.trajectory = TrajectoryData()
        self.mission_start = MissionStartData()
        self.led_control = LedControlState()
        self.thruster_test = ThrusterTestState()
        self.arm_service_available: bool = False
        self.warning_count: int = 0
        self.event_log: list[str] = []

    def log_event(self, text: str) -> None:
        """UIのイベントログへ1行追加する(直近30件のみ保持)。"""
        timestamp = time.strftime("%H:%M:%S")
        self.event_log.append(f"[{timestamp}] {text}")
        if len(self.event_log) > 30:
            self.event_log.pop(0)

    def check_low_voltage_warning(self) -> bool:
        """低電圧が継続しているかチェックする。"""
        is_warning = 0.1 < self.power.voltage <= self.config.volt_warn_low
        if is_warning:
            self.warning_count += 1
        else:
            self.warning_count = 0
        return self.warning_count >= 3


app_state = RobotState()


# ==========================================
# Section 3b: Camera MJPEG Stream
# ==========================================
# ui.image().set_source()でbase64を毎回差し替える方式はブラウザが画像を都度
# 再読み込みしてしまいチラつくため、単一の<img>タグにMJPEGストリームを
# 流し込む方式にしている。
async def _mjpeg_frames():
    boundary = b"--frame"
    last_sent = 0.0
    while True:
        await asyncio.sleep(0.1)
        camera = app_state.camera
        if camera.last_update == last_sent or not camera.jpg_bytes:
            continue
        last_sent = camera.last_update
        frame = camera.jpg_bytes
        yield (
            boundary + b"\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Content-Length: " + str(len(frame)).encode("ascii") + b"\r\n\r\n" + frame + b"\r\n"
        )


@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(
        _mjpeg_frames(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


# ==========================================
# Section 4: ROS 2 Logic
# ==========================================
class MonitorNode(Node):
    def __init__(self, state: RobotState):
        super().__init__("bluerov_dashboard_node")
        self.state = state
        self._bridge = CvBridge() if ROS_AVAILABLE else None
        self._min_camera_interval_s = 1.0 / 5.0
        self._axis_pulse_timers: dict[str, threading.Timer] = {}
        self._init_params()
        self._init_subs()
        self._init_pubs()
        self._init_service_client()
        self.create_timer(1.0, self._update_service_status)
        self.create_timer(0.2, self._heartbeat_timer_callback)

    def _init_params(self):
        cfg = self.state.config
        params = {
            "timeout.depth": "timeout_depth",
            "timeout.imu": "timeout_imu",
            "timeout.power": "timeout_power",
            "timeout.gnss": "timeout_gnss",
            "timeout.vehicle_state": "timeout_vehicle_state",
            "timeout.dvl": "timeout_dvl",
            "timeout.camera": "timeout_camera",
            "voltage.warn_low": "volt_warn_low",
            "voltage.warn_critical": "volt_warn_critical",
        }
        for ros_param, cfg_attr in params.items():
            val = self.declare_parameter(ros_param, getattr(cfg, cfg_attr)).value
            setattr(cfg, cfg_attr, val)

    def _init_subs(self):
        self.create_subscription(Depth, "depth", self.cb_depth, 10)
        self.create_subscription(IMU, "imu", self.cb_imu, 10)
        self.create_subscription(PowerState, "power_state", self.cb_power, 10)
        self.create_subscription(Gnss, "gnss", self.cb_gnss, 10)
        self.create_subscription(VehicleState, "vehicle_state", self.cb_vehicle_state, 10)
        self.create_subscription(DVL75, "dvl", self.cb_dvl, 10)
        self.create_subscription(
            Image, "image_raw", self.cb_camera, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "odom", self.cb_odom, 10)
        self.create_subscription(
            BoolStamped, "mission_start_trigger", self.cb_mission_start, 10
        )

    def _init_pubs(self):
        self._led_publisher = self.create_publisher(LED, "led", 10)
        self._tilt_publisher = self.create_publisher(Int32Stamped, "camera_tilt", 10)
        self._wrench_publisher = self.create_publisher(WrenchStamped, "robot_force", 10)
        self._heartbeat_publisher = self.create_publisher(Bool, "heartbeat", 10)
        self._mission_start_publisher = self.create_publisher(
            BoolStamped, "mission_start_trigger", 10
        )

    def _init_service_client(self):
        self._set_armed_client = self.create_client(SetBool, "set_armed")

    def _update_service_status(self):
        if self._set_armed_client is None:
            return
        self.state.arm_service_available = self._set_armed_client.service_is_ready()

    def _heartbeat_timer_callback(self) -> None:
        """manual_control_enabled中だけheartbeatを送り続ける。

        送信を止めればmavlink_driver側のcontrol_heartbeat_timeout_sで
        自動的にスラスターがニュートラルへ戻る安全設計。
        """
        if not self.state.thruster_test.manual_control_enabled:
            return
        message = Bool()
        message.data = True
        self._heartbeat_publisher.publish(message)

    def _update_common(self, data_obj: SensorData, msg) -> None:
        data_obj.last_update = time.time()
        data_obj.status_id = msg.status.id

    def cb_depth(self, msg) -> None:
        d = self.state.depth
        self._update_common(d, msg)
        d.depth = msg.depth
        d.temp = msg.temperature

    def cb_imu(self, msg) -> None:
        d = self.state.imu
        self._update_common(d, msg)
        d.roll = msg.orient.x
        d.pitch = msg.orient.y
        d.yaw = msg.orient.z

    def cb_power(self, msg) -> None:
        d = self.state.power
        self._update_common(d, msg)
        d.voltage = msg.act_voltage
        d.current = msg.act_current
        d.power = msg.act_power

    def cb_gnss(self, msg) -> None:
        d = self.state.gnss
        self._update_common(d, msg)
        d.fix_ok = msg.status.id == 0
        d.latitude = msg.fix.latitude
        d.longitude = msg.fix.longitude
        d.altitude = msg.fix.altitude

    def cb_vehicle_state(self, msg) -> None:
        d = self.state.vehicle
        self._update_common(d, msg)
        d.connected = msg.connected
        d.armed = msg.armed
        d.custom_mode = msg.custom_mode
        d.system_status = msg.system_status

    def cb_dvl(self, msg) -> None:
        d = self.state.dvl
        d.last_update = time.time()
        d.status_id = 0 if msg.dvext_valid else 2
        d.velocity_north = msg.velocity_north
        d.velocity_east = msg.velocity_east
        d.velocity_up = msg.velocity_up
        d.altitude = msg.altitude
        d.confidence = msg.confidence
        d.bottom_lock = msg.bottom_lock
        d.speed_norm = math.sqrt(msg.velocity_north**2 + msg.velocity_east**2)

    def cb_camera(self, msg) -> None:
        now = time.time()
        if now - self.state.camera.last_update < self._min_camera_interval_s:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as error:
            self.get_logger().error(f"カメラ画像の変換に失敗しました: {error}")
            return

        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ok:
            return

        self.state.camera.jpg_bytes = encoded.tobytes()
        self.state.camera.last_update = now

    def cb_odom(self, msg) -> None:
        d = self.state.trajectory
        d.last_update = time.time()
        d.x = msg.pose.position.x
        d.y = msg.pose.position.y
        d.depth_m = msg.pose.position.z_depth
        d.yaw = msg.pose.orientation.z
        d.points.append((d.x, d.y))

    def cb_mission_start(self, msg) -> None:
        d = self.state.mission_start
        d.last_update = time.time()
        if msg.data and not d.triggered:
            d.triggered_at = time.time()
            self.state.log_event("Mission start trigger received (reed switch)")
        d.triggered = msg.data

    def publish_led(self, left_pwm: int, right_pwm: int) -> None:
        message = LED()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status.id = 0
        message.left = int(left_pwm)
        message.right = int(right_pwm)
        self._led_publisher.publish(message)

    def publish_tilt(self, angle_deg: float) -> None:
        message = Int32Stamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status.id = 0
        message.data = int(angle_deg)
        self._tilt_publisher.publish(message)

    def publish_mission_start_trigger(self) -> None:
        message = BoolStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "bluerov_dashboard"
        message.status.id = 0
        message.data = True
        self._mission_start_publisher.publish(message)

    def _publish_wrench(self, fx: float, fy: float, fz: float, tx: float, ty: float, tz: float) -> None:
        message = WrenchStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.wrench.force.x = fx
        message.wrench.force.y = fy
        message.wrench.force.z = fz
        message.wrench.torque.x = tx
        message.wrench.torque.y = ty
        message.wrench.torque.z = tz
        self._wrench_publisher.publish(message)

    def pulse_axis(self, axis: str, sign: float) -> None:
        """指定軸へ一定時間だけ力/トルクを与え、その後ゼロへ戻す(スラスターテスト用)。"""
        if not self.state.thruster_test.manual_control_enabled:
            ui.notify("先に「Manual Control」を有効にしてください", type="warning")
            return

        force = self.state.thruster_test.test_force_n * sign
        torque = self.state.thruster_test.test_torque_nm * sign
        axis_wrench = {
            "surge": (force, 0.0, 0.0, 0.0, 0.0, 0.0),
            "sway": (0.0, force, 0.0, 0.0, 0.0, 0.0),
            "heave": (0.0, 0.0, force, 0.0, 0.0, 0.0),
            "roll": (0.0, 0.0, 0.0, torque, 0.0, 0.0),
            "yaw": (0.0, 0.0, 0.0, 0.0, 0.0, torque),
        }
        if axis not in axis_wrench:
            raise ValueError(f"unknown axis: {axis}")

        existing_timer = self._axis_pulse_timers.get(axis)
        if existing_timer is not None:
            existing_timer.cancel()

        self._publish_wrench(*axis_wrench[axis])
        self.state.log_event(f"Thruster pulse: {axis} sign={sign:+.0f}")

        duration = self.state.thruster_test.pulse_duration_s
        timer = threading.Timer(duration, self._publish_wrench, args=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self._axis_pulse_timers[axis] = timer
        timer.start()

    def emergency_stop(self) -> None:
        """全軸ゼロ・ハートビート停止・Disarmを即座に行う。"""
        for timer in self._axis_pulse_timers.values():
            timer.cancel()
        self._axis_pulse_timers.clear()
        self._publish_wrench(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.state.thruster_test.manual_control_enabled = False
        self.call_set_armed(False)
        self.state.log_event("EMERGENCY STOP: disarm + zero wrench")

    def call_set_armed(self, armed: bool) -> None:
        if self._set_armed_client is None or not self._set_armed_client.service_is_ready():
            ui.notify("set_armedサービスに接続できません", type="warning")
            return
        request = SetBool.Request()
        request.data = armed
        future = self._set_armed_client.call_async(request)
        future.add_done_callback(lambda f: self._on_set_armed_done(f, armed))

    def _on_set_armed_done(self, future, armed: bool) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"set_armed呼び出しに失敗しました: {error}")
            self.state.log_event(f"set_armed({armed}) call failed: {error}")
            return
        if not response.success:
            self.get_logger().warning(f"set_armed({armed})が拒否されました: {response.message}")
            self.state.log_event(f"set_armed({armed}) rejected: {response.message}")
        else:
            self.state.log_event(f"set_armed({armed}) confirmed")


node_instance: Optional[MonitorNode] = None


# ==========================================
# Section 5: UI Components
# ==========================================
class CyberCard(ui.card):
    """モダンなグラスモーフィズムカードコンポーネント"""

    def __init__(self, no_padding=False):
        super().__init__()
        self.classes("cyber-card w-full")
        self.classes("p-0" if no_padding else "p-6")

    def update_status(self, status_id: int, is_timeout: bool) -> None:
        self.classes(remove="status-normal status-warn status-error status-timeout")
        if is_timeout:
            self.classes(add="status-timeout")
        elif status_id == 2:
            self.classes(add="status-error")
        elif status_id == 1:
            self.classes(add="status-warn")
        else:
            self.classes(add="status-normal")


def label_header(text: str, icon: str = None) -> None:
    with (
        ui.row()
        .classes("items-center gap-2 mb-4 w-full border-b pb-2")
        .style(f"border-color: {UIColors.BORDER}")
    ):
        if icon:
            ui.icon(icon, size="1.5rem").style(f"color: {UIColors.TEXT_SUB}")
        ui.label(text).classes("text-lg font-bold tracking-wider uppercase").style(
            f"color: {UIColors.TEXT_MAIN}"
        )


def value_item(label: str, unit: str, bind_obj, bind_attr, fmt="{:.2f}", color=UIColors.NORMAL):
    with ui.column().classes("items-start min-w-[100px]"):
        ui.label(label).classes("text-ms uppercase tracking-widest font-bold mb-1").style(
            f"color: {UIColors.TEXT_SUB}"
        )
        with ui.row().classes("items-baseline gap-1"):
            lbl = ui.label().bind_text_from(bind_obj, bind_attr, backward=lambda x: fmt.format(x))
            lbl.classes("text-4xl font-bold stat-value leading-none")
            lbl.style(f"color: {color}; text-shadow: 0 0 10px var(--shadow-color);")
            ui.label(unit).classes("text-sm font-bold mb-1").style(f"color: {UIColors.TEXT_SUB}")


def circular_gauge_modern(label, bind_obj, bind_attr, min_val, max_val, unit, icon_name, color, fmt="{:.1f}"):
    with ui.column().classes("items-center relative"):
        with ui.circular_progress(min=min_val, max=max_val, show_value=False).props(
            f'size="110px" thickness=0.1 color="{color}" track-color="grey-4"'
        ) as p:
            with ui.column().classes("absolute-center items-center gap-0"):
                ui.icon(icon_name, size="1.4rem").style(
                    f"color: {color}; opacity: 0.8; margin-bottom: -2px;"
                )
                txt = ui.label().bind_text_from(bind_obj, bind_attr, backward=lambda x: fmt.format(x))
                txt.classes("text-3xl font-bold stat-value leading-none")
                txt.style(f"color: {UIColors.TEXT_MAIN}; text-shadow: 0 2px 4px var(--shadow-color);")

        with ui.row().classes("items-baseline gap-1 mt-2"):
            ui.label(label).classes("text-sm font-bold uppercase tracking-wide").style(
                f"color: {UIColors.TEXT_SUB}"
            )
            ui.label(f" [{unit}]").classes("text-xs font-bold").style(f"color: {UIColors.TEXT_SUB}")
        return p


def status_led_modern(label: str, bind_obj, bind_attr, invert=False):
    with (
        ui.row()
        .classes("w-full items-center justify-between py-3 border-b last:border-0")
        .style(f"border-color: {UIColors.BORDER}")
    ):
        ui.label(label).classes("text-lg font-bold").style(f"color: {UIColors.TEXT_SUB}")
        with ui.row().classes("items-center gap-3"):
            status_text = ui.label("OFF").classes("text-xs font-bold tracking-widest")
            led = ui.element("div").classes("w-3 h-3 rounded-full transition-all duration-300")

        def update():
            is_on = getattr(bind_obj, bind_attr)
            if invert:
                is_on = not is_on
            if is_on:
                led.style(f"background-color: {UIColors.NORMAL}; box-shadow: 0 0 12px {UIColors.NORMAL};")
                status_text.text = "ACTIVE"
                status_text.style(f"color: {UIColors.NORMAL};")
            else:
                led.style("background-color: #64748b; box-shadow: none;")
                status_text.text = "OFF"
                status_text.style(f"color: {UIColors.TEXT_SUB};")

        return update


# ==========================================
# Section 6: UI Layout Composers
# ==========================================
def render_header(dark_mode):
    with (
        ui.header()
        .classes(
            "bg-slate-900/80 backdrop-blur-md border-b border-slate-700 h-16 px-6 items-center sticky top-0 z-50"
        )
        .style("background-color: var(--card-bg); border-color: var(--border-color);")
    ):
        with ui.row().classes("items-center gap-4"):
            ui.icon("directions_boat", size="2rem").style(f"color: {UIColors.NEON_CYAN}")
            with ui.column().classes("gap-0"):
                ui.label("BLUEROV SYSTEM").classes(
                    "text-xl font-bold tracking-[0.2em] leading-none"
                ).style(f"color: {UIColors.TEXT_MAIN}")
                ui.label("OPERATIONAL DASHBOARD").classes(
                    "text-[10px] font-mono tracking-widest"
                ).style(f"color: {UIColors.NEON_CYAN}")

        ui.space()

        with ui.row().classes("items-center gap-4"):
            with (
                ui.button(icon="dark_mode", on_click=dark_mode.toggle)
                .props("flat round dense")
                .classes("text-slate-400")
            ):
                ui.tooltip("Toggle Light/Dark Mode")

            with (
                ui.row()
                .classes("items-center gap-2 px-3 py-1 rounded border")
                .style("background-color: var(--bg-base); border-color: var(--border-color)")
            ):
                ui.element("div").classes("w-2 h-2 rounded-full animate-pulse").style(
                    f"background-color: {UIColors.NORMAL}"
                )
                ui.label("ROS2 CONNECTED" if ROS_AVAILABLE else "UI DEMO MODE").classes(
                    "text-xs font-bold"
                ).style(f"color: {UIColors.NORMAL}")


def render_camera_column(state: RobotState) -> dict:
    refs = {}
    with ui.column().classes("col-span-1 lg:col-span-2 gap-6"):
        card_cam = CyberCard(no_padding=True)
        refs["card_cam"] = card_cam
        with card_cam:
            with ui.column().classes("p-4 w-full"):
                label_header("Camera Feed", "videocam")
                refs["camera_image"] = ui.image("/video_feed").classes(
                    "w-full rounded-lg bg-black"
                ).style("aspect-ratio: 16/9; object-fit: contain;")

        card_vehicle = CyberCard()
        refs["card_vehicle"] = card_vehicle
        with card_vehicle:
            label_header("Vehicle Control", "settings_remote")
            with ui.row().classes("w-full justify-around mb-4"):
                refs["led_connected"] = status_led_modern("Connected", state.vehicle, "connected")
                refs["led_armed"] = status_led_modern("Armed", state.vehicle, "armed")

            with ui.row().classes("w-full justify-between px-1 mb-4"):
                value_item(
                    "Mode",
                    "",
                    state.vehicle,
                    "custom_mode",
                    fmt="{:d}",
                    color=UIColors.TEXT_MAIN,
                )
                value_item(
                    "MAV Status",
                    "",
                    state.vehicle,
                    "system_status",
                    fmt="{:d}",
                    color=UIColors.TEXT_SUB,
                )

            with ui.row().classes("w-full gap-3"):

                def on_arm():
                    if node_instance:
                        node_instance.call_set_armed(True)
                        ui.notify("SENT: Arm", type="warning", color=UIColors.WARN)
                    else:
                        ui.notify("ROS Node Not Connected", type="warning")

                def on_disarm():
                    if node_instance:
                        node_instance.call_set_armed(False)
                        ui.notify("SENT: Disarm", type="info", color=UIColors.NEON_CYAN)
                    else:
                        ui.notify("ROS Node Not Connected", type="warning")

                ui.button("ARM", on_click=on_arm).classes(
                    "flex-1 text-lg font-bold tracking-widest h-12 shadow-lg"
                ).props('color="red-6" icon="warning" no-caps')
                ui.button("DISARM", on_click=on_disarm).classes(
                    "flex-1 text-lg font-bold tracking-widest h-12 shadow-lg"
                ).props('color="cyan-4" icon="stop_circle" no-caps')

            def on_emergency_stop():
                if node_instance:
                    node_instance.emergency_stop()
                    ui.notify("EMERGENCY STOP SENT", type="negative")
                else:
                    ui.notify("ROS Node Not Connected", type="warning")

            ui.button("EMERGENCY STOP", on_click=on_emergency_stop).classes(
                "w-full text-lg font-bold tracking-widest h-14 shadow-lg mt-3"
            ).props('color="red-10" icon="dangerous" no-caps')
    return refs


def render_power_depth_column(state: RobotState) -> dict:
    refs = {}
    with ui.column().classes("col-span-1 gap-6"):
        card_pwr = CyberCard()
        refs["card_pwr"] = card_pwr
        with card_pwr:
            label_header("Battery", "battery_charging_full")
            with ui.row().classes("w-full justify-between px-1"):
                value_item("Voltage", "V", state.power, "voltage", color=UIColors.TEXT_MAIN)
                value_item("Current", "A", state.power, "current", color=UIColors.TEXT_SUB)
            with ui.row().classes("w-full justify-between px-1 mt-2"):
                value_item("Power", "W", state.power, "power", color=UIColors.TEXT_SUB)

        card_depth = CyberCard()
        refs["card_depth"] = card_depth
        with card_depth:
            label_header("Depth", "scuba_diving")
            with ui.row().classes("w-full justify-around"):
                refs["gauge_depth"] = circular_gauge_modern(
                    "Depth", state.depth, "depth", 0, 10, "m", "vertical_align_bottom", "#3b82f6", "{:.2f}"
                )
                refs["gauge_wtemp"] = circular_gauge_modern(
                    "Water Temp", state.depth, "temp", 0, 40, "°C", "water", "#06b6d4", "{:.1f}"
                )
    return refs


def render_imu_gnss_column(state: RobotState) -> dict:
    refs = {}
    with ui.column().classes("col-span-1 gap-6"):
        card_imu = CyberCard()
        refs["card_imu"] = card_imu
        with card_imu:
            label_header("IMU (Attitude)", "explore")
            with ui.row().classes("w-full justify-between px-1"):
                value_item("Roll", "deg", state.imu, "roll", "{:.1f}", UIColors.TEXT_MAIN)
                value_item("Pitch", "deg", state.imu, "pitch", "{:.1f}", UIColors.TEXT_MAIN)
                value_item("Yaw", "deg", state.imu, "yaw", "{:.1f}", UIColors.NEON_PURPLE)

        card_gnss = CyberCard()
        refs["card_gnss"] = card_gnss
        with card_gnss:
            label_header("GNSS", "satellite_alt")
            refs["led_gnss_fix"] = status_led_modern("Fix", state.gnss, "fix_ok")
            with ui.row().classes("w-full justify-between px-1 mt-2"):
                value_item("Lat", "deg", state.gnss, "latitude", "{:.6f}", UIColors.TEXT_MAIN)
                value_item("Lon", "deg", state.gnss, "longitude", "{:.6f}", UIColors.TEXT_MAIN)
            with ui.row().classes("w-full px-1 mt-2"):
                value_item("Altitude", "m", state.gnss, "altitude", "{:.2f}", UIColors.TEXT_SUB)
    return refs


def render_dvl_column(state: RobotState) -> dict:
    refs = {}
    with ui.column().classes("col-span-1 gap-6"):
        card_dvl = CyberCard(no_padding=True)
        refs["card_dvl"] = card_dvl
        with card_dvl:
            with ui.column().classes("p-4 w-full items-center"):
                label_header("DVL (Velocity Vector)", "radar")
                with ui.element("div").classes(
                    "relative w-64 h-64 flex items-center justify-center radar-grid my-2 shadow-[inset_0_0_20px_rgba(0,0,0,0.5)]"
                ):
                    refs["arrow_el"] = ui.html(
                        """
                        <svg width="200" height="200" viewBox="-50 -50 100 100" style="filter: drop-shadow(0px 0px 8px rgba(6, 182, 212, 0.6)); overflow: visible;">
                            <path d="M0,-45 L-5,-35 L0,-38 L5,-35 Z" fill="#06b6d4" />
                            <line x1="0" y1="0" x2="0" y2="-38" stroke="#06b6d4" stroke-width="2" stroke-linecap="round" />
                            <circle cx="0" cy="0" r="3" fill="#06b6d4" />
                        </svg>
                        """,
                        sanitize=False,
                    )

                with ui.row().classes("w-full justify-between mt-4 px-2"):
                    with ui.column().classes("items-center"):
                        ui.label("ALTITUDE").classes("text-xs font-bold text-slate-500")
                        ui.label().bind_text_from(
                            state.dvl, "altitude", backward=lambda x: f"{x:.2f} m"
                        ).classes("text-2xl font-bold stat-value text-white")
                    with ui.column().classes("items-center"):
                        ui.label("SOG").classes("text-xs font-bold text-slate-500")
                        ui.label().bind_text_from(
                            state.dvl, "speed_norm", backward=lambda x: f"{x:.2f} m/s"
                        ).classes("text-2xl font-bold stat-value text-cyan-400")

                with ui.row().classes("w-full justify-between mt-2 px-2"):
                    refs["led_bottom_lock"] = status_led_modern(
                        "Bottom Lock", state.dvl, "bottom_lock"
                    )
    return refs


def render_mission_start_column(state: RobotState) -> dict:
    refs = {}
    with ui.column().classes("col-span-1 gap-6"):
        card_mission = CyberCard()
        refs["card_mission"] = card_mission
        with card_mission:
            label_header("Mission Start", "flag_circle")
            refs["led_mission_triggered"] = status_led_modern(
                "Reed Switch", state.mission_start, "triggered"
            )
            refs["mission_elapsed_label"] = (
                ui.label("--")
                .classes("text-3xl font-bold stat-value leading-none mt-3")
                .style(f"color: {UIColors.NEON_CYAN};")
            )
            ui.label("ELAPSED SINCE TRIGGER").classes(
                "text-xs font-bold uppercase tracking-wide"
            ).style(f"color: {UIColors.TEXT_SUB}")

            def on_simulate_trigger():
                if node_instance:
                    node_instance.publish_mission_start_trigger()
                    ui.notify("SENT: Simulated mission start trigger", type="info")
                else:
                    ui.notify("ROS Node Not Connected", type="warning")

            ui.button("SIMULATE TRIGGER", on_click=on_simulate_trigger).classes(
                "w-full font-bold tracking-widest h-10 shadow-lg mt-4"
            ).props('color="purple-5" icon="bolt" no-caps')
    return refs


def render_led_tilt_column(state: RobotState) -> dict:
    refs = {}
    with ui.column().classes("col-span-1 gap-6"):
        card_led = CyberCard()
        refs["card_led"] = card_led
        with card_led:
            label_header("Light Control", "lightbulb")

            with ui.column().classes("w-full mb-1 gap-0"):
                with ui.row().classes("w-full justify-between items-end mb-1"):
                    ui.label("LED").classes("text-sm font-bold").style(
                        f"color: {UIColors.TEXT_SUB}"
                    )
                    ui.label().bind_text_from(state.led_control, "pwm").classes(
                        "text-lg font-mono font-bold"
                    ).style(f"color: {UIColors.NEON_CYAN}")
                ui.slider(min=1100, max=1900, step=10).bind_value(
                    state.led_control, "pwm"
                ).props('color="cyan-4" track-color="grey-8"').classes("w-full")

            def on_set_led():
                if node_instance:
                    node_instance.publish_led(state.led_control.pwm, state.led_control.pwm)
                    ui.notify(f"SENT: LED {state.led_control.pwm}", type="info")
                else:
                    ui.notify("ROS Node Not Connected", type="warning")

            ui.button("SET LIGHT", on_click=on_set_led).classes(
                "w-full font-bold tracking-widest h-10 shadow-lg mt-3"
            ).props('color="cyan-4" icon="wb_incandescent" no-caps')

        card_tilt = CyberCard()
        refs["card_tilt"] = card_tilt
        with card_tilt:
            label_header("Camera Tilt", "videocam")
            ui.label(
                "位置決めサーボではなく連続回転式のため、押している間だけ動き"
                "離すと停止します(絶対角度指定は不可)。"
            ).classes("text-xs font-bold mb-3").style(f"color: {UIColors.WARN}")

            refs["tilt_state_label"] = (
                ui.label("STOPPED")
                .classes("text-sm font-bold tracking-widest mb-3")
                .style(f"color: {UIColors.TEXT_SUB}")
            )

            def on_tilt_start(direction_deg: float, label: str):
                def handler():
                    if node_instance:
                        node_instance.publish_tilt(direction_deg)
                        refs["tilt_state_label"].text = f"MOVING: {label}"
                        refs["tilt_state_label"].style(f"color: {UIColors.WARN};")
                    else:
                        ui.notify("ROS Node Not Connected", type="warning")

                return handler

            def on_tilt_stop():
                if node_instance:
                    node_instance.publish_tilt(90.0)
                    refs["tilt_state_label"].text = "STOPPED"
                    refs["tilt_state_label"].style(f"color: {UIColors.TEXT_SUB};")

            with ui.row().classes("w-full gap-3"):
                up_button = ui.button("▲ UP").classes(
                    "flex-1 font-bold tracking-widest h-14 shadow-lg"
                ).props('color="orange-5" no-caps')
                up_button.on("mousedown", on_tilt_start(180.0, "UP"))
                up_button.on("mouseup", on_tilt_stop)
                up_button.on("mouseleave", on_tilt_stop)
                up_button.on("touchstart", on_tilt_start(180.0, "UP"))
                up_button.on("touchend", on_tilt_stop)

                down_button = ui.button("▼ DOWN").classes(
                    "flex-1 font-bold tracking-widest h-14 shadow-lg"
                ).props('color="cyan-4" no-caps')
                down_button.on("mousedown", on_tilt_start(0.0, "DOWN"))
                down_button.on("mouseup", on_tilt_stop)
                down_button.on("mouseleave", on_tilt_stop)
                down_button.on("touchstart", on_tilt_start(0.0, "DOWN"))
                down_button.on("touchend", on_tilt_stop)

            ui.label(
                "UP/DOWNが逆であれば、mavlink_driverのcamera_angle設定を反転してください。"
            ).classes("text-[10px] mt-2").style(f"color: {UIColors.TEXT_SUB}")
    return refs


def render_thruster_test_column(state: RobotState) -> dict:
    refs = {}
    with ui.column().classes("col-span-1 lg:col-span-2 gap-6"):
        card_thruster = CyberCard()
        refs["card_thruster"] = card_thruster
        with card_thruster:
            label_header("Thruster Test", "settings_input_component")
            ui.label(
                "実際にスラスターが動きます。プロペラガード装着・拘束状態を確認してください。"
            ).classes("text-xs font-bold mb-3").style(f"color: {UIColors.WARN}")

            with ui.row().classes("w-full items-center justify-between mb-3"):
                ui.label("MANUAL CONTROL (heartbeat)").classes("text-sm font-bold").style(
                    f"color: {UIColors.TEXT_SUB}"
                )

                def on_manual_toggle(e):
                    state.thruster_test.manual_control_enabled = e.value
                    if node_instance:
                        node_instance.state.log_event(
                            f"Manual control {'ENABLED' if e.value else 'disabled'}"
                        )

                ui.switch(on_change=on_manual_toggle).bind_value(
                    state.thruster_test, "manual_control_enabled"
                ).props('color="red-6" keep-color size="lg"')

            with ui.row().classes("w-full gap-4 mb-3"):
                with ui.column().classes("flex-1"):
                    ui.label("FORCE (N)").classes("text-xs font-bold").style(
                        f"color: {UIColors.TEXT_SUB}"
                    )
                    ui.number(value=15.0, min=0.1, max=100.0, step=1.0).bind_value(
                        state.thruster_test, "test_force_n"
                    ).props("outlined dense").classes("w-full")
                with ui.column().classes("flex-1"):
                    ui.label("TORQUE (N*m)").classes("text-xs font-bold").style(
                        f"color: {UIColors.TEXT_SUB}"
                    )
                    ui.number(value=2.0, min=0.1, max=20.0, step=0.5).bind_value(
                        state.thruster_test, "test_torque_nm"
                    ).props("outlined dense").classes("w-full")
                with ui.column().classes("flex-1"):
                    ui.label("PULSE (s)").classes("text-xs font-bold").style(
                        f"color: {UIColors.TEXT_SUB}"
                    )
                    ui.number(value=1.0, min=0.2, max=5.0, step=0.1).bind_value(
                        state.thruster_test, "pulse_duration_s"
                    ).props("outlined dense").classes("w-full")

            def make_pulse_handler(axis: str, sign: float):
                def handler():
                    if node_instance:
                        node_instance.pulse_axis(axis, sign)
                    else:
                        ui.notify("ROS Node Not Connected", type="warning")

                return handler

            axis_labels = [
                ("surge", "Surge (Fwd/Back)", "arrow_upward"),
                ("sway", "Sway (L/R)", "arrow_forward"),
                ("heave", "Heave (Up/Down)", "unfold_more"),
                ("roll", "Roll", "sync"),
                ("yaw", "Yaw", "rotate_right"),
            ]
            for axis_name, axis_label, icon_name in axis_labels:
                with ui.row().classes("w-full items-center justify-between py-1"):
                    with ui.row().classes("items-center gap-2"):
                        ui.icon(icon_name, size="1.2rem").style(f"color: {UIColors.TEXT_SUB}")
                        ui.label(axis_label).classes("text-sm font-bold").style(
                            f"color: {UIColors.TEXT_MAIN}"
                        )
                    with ui.row().classes("gap-2"):
                        ui.button("-", on_click=make_pulse_handler(axis_name, -1.0)).props(
                            'color="cyan-4" dense'
                        ).classes("w-12 font-bold")
                        ui.button("+", on_click=make_pulse_handler(axis_name, 1.0)).props(
                            'color="orange-5" dense'
                        ).classes("w-12 font-bold")
    return refs


def render_trajectory_column(state: RobotState) -> dict:
    refs = {}
    with ui.column().classes("col-span-1 gap-6"):
        card_traj = CyberCard(no_padding=True)
        refs["card_traj"] = card_traj
        with card_traj:
            with ui.column().classes("p-4 w-full items-center"):
                label_header("Trajectory (top-down)", "route")
                with ui.element("div").classes(
                    "relative w-64 h-64 flex items-center justify-center radar-grid my-2 shadow-[inset_0_0_20px_rgba(0,0,0,0.5)]"
                ):
                    refs["trajectory_svg"] = ui.html(
                        '<svg width="220" height="220" viewBox="-110 -110 220 220" '
                        'style="overflow: visible;">'
                        '<polyline points="" fill="none" stroke="#06b6d4" stroke-width="2" '
                        'id="traj-line" />'
                        '<circle cx="0" cy="0" r="4" fill="#f97316" id="traj-current" />'
                        "</svg>",
                        sanitize=False,
                    )

                with ui.row().classes("w-full justify-between mt-4 px-2"):
                    value_item("X", "m", state.trajectory, "x", "{:.2f}", UIColors.TEXT_MAIN)
                    value_item("Y", "m", state.trajectory, "y", "{:.2f}", UIColors.TEXT_MAIN)
                value_item(
                    "Depth", "m", state.trajectory, "depth_m", "{:.2f}", UIColors.NEON_CYAN
                )
    return refs


def render_event_log_row(state: RobotState) -> dict:
    refs = {}
    with ui.column().classes("col-span-1 lg:col-span-5 gap-6"):
        card_log = CyberCard()
        refs["card_log"] = card_log
        with card_log:
            label_header("Event Log", "history")
            refs["event_log"] = ui.log(max_lines=100).classes(
                "w-full h-40 font-mono text-xs leading-relaxed bg-black/30 p-2 rounded border"
            ).style(f"border-color: {UIColors.BORDER}; color: {UIColors.NEON_CYAN};")
    return refs


# ==========================================
# Section 7: Main Entry & Loop
# ==========================================
@ui.page("/")
def index():
    ui.add_head_html(f"<style>{CSS_STYLES}</style>")
    dark_mode = ui.dark_mode(value=True)
    ui.colors(primary=UIColors.NEON_CYAN, secondary="#0f172a", accent=UIColors.NORMAL)

    render_header(dark_mode)

    main_layout = ui.element("div").classes(
        "w-full min-h-screen p-4 md:p-6 transition-all duration-300"
    )

    with main_layout:
        with ui.grid().classes("grid-cols-1 lg:grid-cols-5 gap-6 w-full max-w-7xl mx-auto"):
            refs_cam = render_camera_column(app_state)
            refs_pwr = render_power_depth_column(app_state)
            refs_imu = render_imu_gnss_column(app_state)
            refs_dvl = render_dvl_column(app_state)
            refs_mission = render_mission_start_column(app_state)
            refs_led_tilt = render_led_tilt_column(app_state)
            refs_thruster = render_thruster_test_column(app_state)
            refs_traj = render_trajectory_column(app_state)
            refs_log = render_event_log_row(app_state)

    ui_refs = {
        **refs_cam,
        **refs_pwr,
        **refs_imu,
        **refs_dvl,
        **refs_mission,
        **refs_led_tilt,
        **refs_thruster,
        **refs_traj,
        **refs_log,
    }

    last_logged_event_count = 0

    def update_ui():
        nonlocal last_logged_event_count
        state = app_state
        cfg = state.config

        # --- Event log (push only new entries) ---
        new_events = state.event_log[last_logged_event_count:]
        for entry in new_events:
            ui_refs["event_log"].push(entry)
        last_logged_event_count = len(state.event_log)

        # --- Mission start elapsed time ---
        if state.mission_start.triggered:
            elapsed = time.time() - state.mission_start.triggered_at
            ui_refs["mission_elapsed_label"].text = f"{elapsed:5.1f}s"
        else:
            ui_refs["mission_elapsed_label"].text = "--"
        ui_refs["led_mission_triggered"]()

        # --- Trajectory SVG (top-down, meters -> pixels, auto-scaled) ---
        points = state.trajectory.points
        if points:
            max_extent = max(1.0, max(max(abs(px), abs(py)) for px, py in points))
            scale = 90.0 / max_extent
            svg_points = " ".join(f"{px * scale:.1f},{-py * scale:.1f}" for px, py in points)
            cur_x, cur_y = points[-1]
            ui.run_javascript(
                f"""
                const line = document.getElementById('traj-line');
                const cur = document.getElementById('traj-current');
                if (line) line.setAttribute('points', '{svg_points}');
                if (cur) {{
                    cur.setAttribute('cx', '{cur_x * scale:.1f}');
                    cur.setAttribute('cy', '{-cur_y * scale:.1f}');
                }}
                """
            )
        ui_refs["card_traj"].update_status(0, state.trajectory.is_timeout(5.0))

        # --- Gauges ---
        ui_refs["gauge_depth"].value = state.depth.depth
        ui_refs["gauge_wtemp"].value = state.depth.temp

        # --- DVL Arrow (0deg = North) ---
        angle = math.degrees(math.atan2(state.dvl.velocity_east, state.dvl.velocity_north))
        scale = min(max(state.dvl.speed_norm / 0.5, 0.2), 1.4)
        ui_refs["arrow_el"].style(
            f"transform: rotate({angle}deg) scale({scale}); "
            "transition: transform 0.2s cubic-bezier(0.4, 0, 0.2, 1);"
        )

        # --- LEDs ---
        ui_refs["led_connected"]()
        ui_refs["led_armed"]()
        ui_refs["led_gnss_fix"]()
        ui_refs["led_bottom_lock"]()

        # --- Card status borders ---
        ui_refs["card_vehicle"].update_status(
            2 if not state.vehicle.connected else 0,
            state.vehicle.is_timeout(cfg.timeout_vehicle_state),
        )
        ui_refs["card_pwr"].update_status(
            state.power.status_id, state.power.is_timeout(cfg.timeout_power)
        )
        ui_refs["card_depth"].update_status(
            state.depth.status_id, state.depth.is_timeout(cfg.timeout_depth)
        )
        ui_refs["card_imu"].update_status(
            state.imu.status_id, state.imu.is_timeout(cfg.timeout_imu)
        )
        ui_refs["card_gnss"].update_status(
            state.gnss.status_id, state.gnss.is_timeout(cfg.timeout_gnss)
        )
        ui_refs["card_dvl"].update_status(
            state.dvl.status_id, state.dvl.is_timeout(cfg.timeout_dvl)
        )
        ui_refs["card_cam"].update_status(0, state.camera.is_timeout(cfg.timeout_camera))
        ui_refs["card_thruster"].update_status(
            1 if state.thruster_test.manual_control_enabled else 0, False
        )
        ui_refs["card_mission"].update_status(0 if state.mission_start.triggered else 1, False)

        # --- Global low-voltage alert ---
        if state.check_low_voltage_warning():
            main_layout.classes(add="global-alert")
        else:
            main_layout.classes(remove="global-alert")

    ui.timer(0.1, update_ui)


def ros_thread():
    global node_instance
    if not ROS_AVAILABLE:
        return

    rclpy.init()
    node_instance = MonitorNode(app_state)
    try:
        rclpy.spin(node_instance)
    except Exception:
        pass
    finally:
        if node_instance:
            node_instance.destroy_node()
        rclpy.shutdown()


def main():
    t = threading.Thread(target=ros_thread, daemon=True)
    t.start()

    # KYUBIC本体の`dashboard`パッケージが同一マシンで動く場合に備え、
    # ポート8080(dashboard)と衝突しない8090を使う。
    ui.run(title="BLUEROV SYSTEM", port=8090, reload=False, dark=True, show=False)


if __name__ == "__main__":
    main()
