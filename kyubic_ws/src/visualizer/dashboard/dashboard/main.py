import math
import threading
import time
from dataclasses import dataclass
from typing import Optional, Callable

from nicegui import ui, app

# ==========================================
# Section 1: ROS 2 Imports & Mocking
# ==========================================
# ROS環境がない場所でもUI開発ができるように、インポートエラーをハンドリングします
try:
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.msg import SetParametersResult
    from std_msgs.msg import Header, String
    from common_msgs.msg import Status
    from driver_msgs.msg import (
        DVL,
        Depth,
        Environment,
        IMU,
        LED,
        PowerState,
        SystemSwitch,
        SystemStatus,
        BoolStamped,
        Int32Stamped,
    )

    ROS_AVAILABLE = True
except ImportError:
    # ダミー定義（ROS環境外での動作確認用）
    ROS_AVAILABLE = False
    print("Warning: ROS 2 libraries not found. Running in UI-only mode.")

    class Node:
        def __init__(self, name):
            pass

        def declare_parameter(self, name, value):
            return type("obj", (), {"value": value})

        def create_subscription(self, *args):
            pass

        def create_publisher(self, *args):
            return type("pub", (), {"publish": lambda x: None})

        def get_clock(self):
            return type("clock", (), {"now": lambda: type("t", (), {"to_msg": lambda: 0})})

        def add_on_set_parameters_callback(self, cb):
            pass

        def destroy_node(self):
            pass

    class SetParametersResult:
        def __init__(self, successful):
            self.successful = successful

    # ダミーメッセージクラス生成
    def create_dummy_msg():
        return type("Msg", (), {"status": type("S", (), {"id": 0})})

    DVL = Depth = Environment = IMU = PowerState = SystemSwitch = SystemStatus = BoolStamped = (
        create_dummy_msg
    )


# ==========================================
# Section 2: Styles & Constants
# ==========================================
class UIColors:
    """UIで使用するカラーパレット定数（CSS変数への参照）"""

    BG_BASE = "var(--bg-base)"
    CARD_BG = "var(--card-bg)"
    TEXT_MAIN = "var(--text-main)"
    TEXT_SUB = "var(--text-sub)"
    BORDER = "var(--border-color)"

    # Status Colors
    NORMAL = "var(--color-normal)"
    WARN = "var(--color-warn)"
    ERROR = "var(--color-error)"
    NEON_CYAN = "var(--color-cyan)"
    NEON_PURPLE = "var(--color-purple)"


CSS_STYLES = """
    @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Noto+Sans+JP:wght@300;500;700&display=swap');

    :root {
        /* Light Mode Defaults */
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
        /* Dark Mode Overrides */
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

    /* Status Indicators */
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
    timeout_power: float = 3.0
    timeout_env: float = 3.0
    timeout_sys_status: float = 3.0
    timeout_dvl: float = 3.0
    timeout_imu: float = 3.0
    timeout_depth: float = 3.0
    volt_warn_low: float = 13.0
    volt_warn_high_start: float = 17.0
    volt_warn_high_end: float = 19.8


@dataclass
class SensorData:
    last_update: float = 0.0
    status_id: int = 0

    def is_timeout(self, timeout_sec: float) -> bool:
        return (time.time() - self.last_update) > timeout_sec


@dataclass
class PowerData(SensorData):
    log_volt: float = 0.0
    log_curr: float = 0.0
    act_volt: float = 0.0
    act_curr: float = 0.0


@dataclass
class DvlData(SensorData):
    vel_x: float = 0.0
    vel_y: float = 0.0
    altitude: float = 0.0
    speed_norm: float = 0.0


@dataclass
class EnvData(SensorData):
    temp: float = 0.0
    humid: float = 0.0
    press: float = 0.0


@dataclass
class ImuData(SensorData):
    orient_z: float = 0.0
    temp: float = 0.0


@dataclass
class DepthData(SensorData):
    depth: float = 0.0
    temp: float = 0.0


@dataclass
class SystemStatusData(SensorData):
    jetson: bool = False
    actuator_power: bool = False
    logic_relay: bool = False
    usb_power: bool = False


@dataclass
class SwitchState:
    jetson: bool = True
    dvl: bool = False
    com: bool = True
    ex1: bool = False
    ex8: bool = False
    actuator: bool = False
    status_id: int = 0


class RobotState:
    """アプリケーション全体の状態を保持するクラス"""

    def __init__(self):
        self.config = SystemConfig()
        self.power = PowerData()
        self.dvl = DvlData()
        self.env = EnvData()
        self.imu = ImuData()
        self.depth = DepthData()
        self.sys_stat = SystemStatusData()
        self.switches = SwitchState()

        self.tilt_angle: int = 90

        self.led_right: int = 1100  # (1100:OFF ~ 1900:MAX)
        self.led_left: int = 1100

        self.topic_alive_ctrl: bool = False  # System Control
        self.topic_alive_tilt: bool = False  # Tilt Serve
        self.topic_alive_led: bool = False  # LED Control

        self.voice_events: list[dict[str]] = []  # {'id': int, 'text': str, 'time': str}
        self.event_counter: int = 0

        self.warning_count: int = 0

    def check_global_voltage_warning(self) -> bool:
        """電圧異常が継続しているかチェック"""

        def _is_warn(v):
            return (v > 0.1 and v <= self.config.volt_warn_low) or (
                self.config.volt_warn_high_start <= v <= self.config.volt_warn_high_end
            )

        is_warning = _is_warn(self.power.log_volt) or _is_warn(self.power.act_volt)
        if is_warning:
            self.warning_count += 1
        else:
            self.warning_count = 0
        return self.warning_count >= 3


# グローバルな状態インスタンス
app_state = RobotState()


# ==========================================
# Section 4: ROS 2 Logic
# ==========================================
class MonitorNode(Node):
    def __init__(self, state: RobotState):
        super().__init__("web_monitor_node")
        self.state = state
        self._init_params()
        self._init_subs_pubs()
        self._init_timers()

    def _init_params(self):
        cfg = self.state.config
        # パラメータ定義と読み込み
        params = {
            "timeout.power": "timeout_power",
            "timeout.env": "timeout_env",
            "timeout.sys_status": "timeout_sys_status",
            "timeout.dvl": "timeout_dvl",
            "timeout.imu": "timeout_imu",
            "timeout.depth": "timeout_depth",
            "voltage.warn_low": "volt_warn_low",
            "voltage.warn_high_start": "volt_warn_high_start",
            "voltage.warn_high_end": "volt_warn_high_end",
        }
        for ros_param, cfg_attr in params.items():
            val = self.declare_parameter(ros_param, getattr(cfg, cfg_attr)).value
            setattr(cfg, cfg_attr, val)

        self.add_on_set_parameters_callback(self.cb_params)

    def _init_subs_pubs(self):
        self.create_subscription(DVL, "dvl", self.cb_dvl, 10)
        self.create_subscription(Depth, "depth", self.cb_depth, 10)
        self.create_subscription(Environment, "environment", self.cb_env, 10)
        self.create_subscription(IMU, "imu", self.cb_imu, 10)
        self.create_subscription(PowerState, "power_state", self.cb_power, 10)
        self.create_subscription(SystemStatus, "system_status", self.cb_sys_status, 10)
        self.create_subscription(String, "talker", self.cb_talker, 10)

        self.pub_sys_switch = self.create_publisher(SystemSwitch, "system_switch", 10)
        self.pub_imu_reset = self.create_publisher(BoolStamped, "imu_reset", 10)
        self.pub_tilt = self.create_publisher(Int32Stamped, "tilt_servo", 10)
        self.pub_led = self.create_publisher(LED, "led", 10)

    def _init_timers(self):
        self.create_timer(1.0, self.update_topic_status)

    def cb_params(self, params):
        cfg = self.state.config
        param_map = {
            "timeout.power": "timeout_power",
            "timeout.env": "timeout_env",
            "timeout.sys_status": "timeout_sys_status",
            "timeout.dvl": "timeout_dvl",
            "timeout.imu": "timeout_imu",
            "timeout.depth": "timeout_depth",
            "voltage.warn_low": "volt_warn_low",
            "voltage.warn_high_start": "volt_warn_high_start",
            "voltage.warn_high_end": "volt_warn_high_end",
        }
        for p in params:
            if p.name in param_map:
                setattr(cfg, param_map[p.name], p.value)
        return SetParametersResult(successful=True)

    def _update_common(self, data_obj: SensorData, msg):
        data_obj.last_update = time.time()
        data_obj.status_id = msg.status.id

    def cb_dvl(self, msg):
        d = self.state.dvl
        self._update_common(d, msg)
        d.vel_x = msg.velocity.x
        d.vel_y = msg.velocity.y
        d.altitude = msg.altitude
        d.speed_norm = math.sqrt(msg.velocity.x**2 + msg.velocity.y**2)

    def cb_depth(self, msg):
        d = self.state.depth
        self._update_common(d, msg)
        d.depth = msg.depth
        d.temp = msg.temperature

    def cb_env(self, msg):
        d = self.state.env
        self._update_common(d, msg)
        d.temp = msg.temperature
        d.humid = msg.humidity
        d.press = msg.pressure

    def cb_imu(self, msg):
        d = self.state.imu
        self._update_common(d, msg)
        d.orient_z = msg.orient.z
        d.temp = msg.temperature

    def cb_power(self, msg):
        d = self.state.power
        self._update_common(d, msg)
        d.log_volt = msg.log_voltage
        d.log_curr = msg.log_current
        d.act_volt = msg.act_voltage
        d.act_curr = msg.act_current

    def cb_sys_status(self, msg):
        d = self.state.sys_stat
        self._update_common(d, msg)
        d.jetson = msg.jetson
        d.actuator_power = msg.actuator_power
        d.logic_relay = msg.logic_relay
        d.usb_power = msg.usb_power

    def cb_talker(self, msg):
        text = msg.data
        if not text:
            return

        timestamp = time.strftime("%H:%M:%S")

        self.state.event_counter += 1
        new_event = {"id": self.state.event_counter, "text": text, "time": timestamp}
        self.state.voice_events.append(new_event)

        # 最新20件だけ保持
        if len(self.state.voice_events) > 20:
            self.state.voice_events.pop(0)

    def publish_switch_command(self):
        s = self.state.switches
        msg = SystemSwitch()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.id = 0
        msg.jetson = s.jetson
        msg.dvl = s.dvl
        msg.com = s.com
        msg.ex1 = s.ex1
        msg.ex8 = s.ex8
        msg.actuator = s.actuator
        self.pub_sys_switch.publish(msg)

    def publish_imu_reset(self):
        msg = BoolStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = True
        self.pub_imu_reset.publish(msg)
        ui.notify("IMU Zero-Reset Command Sent", type="info", color=UIColors.NEON_CYAN)

    def publish_tilt_command(self):
        msg = Int32Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.id = 0
        msg.data = int(self.state.tilt_angle)
        self.pub_tilt.publish(msg)

    def publish_led_command(self):
        msg = LED()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.id = 0
        msg.right = int(self.state.led_right)
        msg.left = int(self.state.led_left)
        self.pub_led.publish(msg)

    def _is_remote_node_alive(self, topic_name: str, mode: str) -> bool:
        """
        指定したトピックに相手がいるか確認する
        :param topic_name: トピック名
        :param mode: 'sub' (自分が受信側=相手はPub) or 'pub' (自分が送信側=相手はSub)
        :return: 接続確認できればTrue
        """
        # ダミーモード(ROS無し)の場合は常にTrue
        if not hasattr(self, "count_publishers"):
            return True

        resolved_name = self.resolve_topic_name(topic_name)

        try:
            if mode == "sub":
                # 相手(Publisher)がいるか数える
                return self.count_publishers(resolved_name) > 0
            elif mode == "pub":
                # 相手(Subscriber)がいるか数える
                return self.count_subscribers(resolved_name) > 0
            else:
                return False
        except Exception:
            return False

    def update_topic_status(self):
        """定期実行されるステータス更新関数"""

        # 1. System Contril
        self.state.topic_alive_ctrl = (
            0 if self._is_remote_node_alive(topic_name="system_switch", mode="pub") else 2
        )

        # 2. Camera Tilt
        self.state.topic_alive_tilt = (
            0 if self._is_remote_node_alive(topic_name="tilt_servo", mode="pub") else 2
        )

        # 3. LED Control
        self.state.topic_alive_led = (
            0 if self._is_remote_node_alive(topic_name="led", mode="pub") else 2
        )


# Global Node Reference (UI callbacks need this)
node_instance: Optional[MonitorNode] = None


# ==========================================
# Section 5: UI Components
# ==========================================
class CyberCard(ui.card):
    """モダンなグラスモーフィズムカードコンポーネント"""

    def __init__(self, no_padding=False):
        super().__init__()
        self.classes("cyber-card w-full")
        if not no_padding:
            self.classes("p-6")
        else:
            self.classes("p-0")

    def update_status(self, status_id: int, is_timeout: bool):
        self.classes(remove="status-normal status-warn status-error status-timeout")
        if is_timeout:
            self.classes(add="status-timeout")
        elif status_id == 2:
            self.classes(add="status-error")
        elif status_id == 1:
            self.classes(add="status-warn")
        else:
            self.classes(add="status-normal")


def label_header(text: str, icon: str = None):
    """カードのヘッダーを表示"""
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
    """数値とラベル、単位を表示する標準的なウィジェット"""
    with ui.column().classes("items-start min-w-[100px]"):
        ui.label(label).classes("text-ms uppercase tracking-widest font-bold mb-1").style(
            f"color: {UIColors.TEXT_SUB}"
        )
        with ui.row().classes("items-baseline gap-1"):
            lbl = ui.label().bind_text_from(bind_obj, bind_attr, backward=lambda x: fmt.format(x))
            lbl.classes("text-5xl font-bold stat-value leading-none")
            lbl.style(f"color: {color}; text-shadow: 0 0 10px var(--shadow-color);")
            ui.label(unit).classes("text-sm font-bold mb-1").style(f"color: {UIColors.TEXT_SUB}")


def circular_gauge_modern(
    label: str,
    bind_obj,
    bind_attr,
    min_val,
    max_val,
    unit,
    icon_name,
    color,
    fmt="{:.1f}",
):
    """円形プログレスバーを用いたゲージ表示"""
    with ui.column().classes("items-center relative"):
        with ui.circular_progress(min=min_val, max=max_val, show_value=False).props(
            f'size="110px" thickness=0.1 color="{color}" track-color="grey-4"'
        ) as p:
            with ui.column().classes("absolute-center items-center gap-0"):
                ui.icon(icon_name, size="1.4rem").style(
                    f"color: {color}; opacity: 0.8; margin-bottom: -2px;"
                )
                txt = ui.label().bind_text_from(
                    bind_obj, bind_attr, backward=lambda x: fmt.format(x)
                )
                txt.classes("text-3xl font-bold stat-value leading-none")
                txt.style(
                    f"color: {UIColors.TEXT_MAIN}; text-shadow: 0 2px 4px var(--shadow-color);"
                )

        with ui.row().classes("items-baseline gap-1 mt-2"):
            ui.label(label).classes("text-sm font-bold uppercase tracking-wide").style(
                f"color: {UIColors.TEXT_SUB}"
            )
            ui.label(f" [{unit}]").classes("text-xs font-bold").style(f"color: {UIColors.TEXT_SUB}")
        return p


def control_switch_modern(label: str, state_obj: SwitchState, attr_name: str):
    """トグルスイッチ"""
    with (
        ui.row()
        .classes("w-full items-center justify-between py-3 border-b last:border-0")
        .style(f"border-color: {UIColors.BORDER}")
    ):
        ui.label(label).classes("text-xl font-bold stat-value").style(
            f"color: {UIColors.TEXT_MAIN}"
        )

        def on_change(e):
            setattr(state_obj, attr_name, e.value)

        s = ui.switch(on_change=on_change).bind_value_from(state_obj, attr_name)
        s.props('color="cyan-4" keep-color size="lg" dense')
        s.classes("mr-2")


def status_led_modern(label: str, bind_obj, bind_attr) -> Callable[[], None]:
    """LEDインジケーター（状態監視用）。更新関数を返す。"""
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
            if is_on:
                led.style(
                    f"background-color: {UIColors.NORMAL}; box-shadow: 0 0 12px {UIColors.NORMAL};"
                )
                status_text.text = "ACTIVE"
                status_text.style(f"color: {UIColors.NORMAL};")
            else:
                led.style(f"background-color: #64748b; box-shadow: none;")
                status_text.text = "OFF"
                status_text.style(f"color: {UIColors.TEXT_SUB};")

        return update


# ==========================================
# Section 6: UI Layout Composers
# ==========================================
def render_header(dark_mode):
    """ヘッダー領域の描画"""
    with (
        ui.header()
        .classes(
            "bg-slate-900/80 backdrop-blur-md border-b border-slate-700 h-16 px-6 items-center sticky top-0 z-50"
        )
        .style("background-color: var(--card-bg); border-color: var(--border-color);")
    ):
        with ui.row().classes("items-center gap-4"):
            ui.icon("precision_manufacturing", size="2rem").style(f"color: {UIColors.NEON_CYAN}")
            with ui.column().classes("gap-0"):
                ui.label("KYUBIC SYSTEM").classes(
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


def render_power_column(state: RobotState) -> dict:
    """左カラム（Power & Env）の構築"""
    refs = {}
    with ui.column().classes("col-span-1 gap-6"):
        # Power Module
        card_pwr = CyberCard()
        refs["card_pwr"] = card_pwr
        with card_pwr:
            label_header("Power Distribution", "bolt")
            # Logic Power
            ui.label("LOGIC CIRCUIT").classes(
                "text-ms font-bold text-cyan-500 mb-2 pl-1 border-l-2 border-cyan-500"
            )
            with ui.row().classes("w-full justify-between mb-4 px-1"):
                value_item("Voltage", "V", state.power, "log_volt", color=UIColors.TEXT_MAIN)
                value_item("Current", "A", state.power, "log_curr", color=UIColors.TEXT_SUB)

            # Actuator Power
            ui.label("ACTUATOR CIRCUIT").classes(
                "text-ms font-bold text-orange-500 mb-2 pl-1 border-l-2 border-orange-500"
            )
            with ui.row().classes("w-full justify-between px-1"):
                value_item("Voltage", "V", state.power, "act_volt", color=UIColors.TEXT_MAIN)
                value_item("Current", "A", state.power, "act_curr", color=UIColors.TEXT_SUB)

        # Environment Module
        card_env = CyberCard()
        refs["card_env"] = card_env
        with card_env:
            label_header("Atmospherics", "thermostat")
            with ui.row().classes("w-full justify-around"):
                refs["gauge_air"] = circular_gauge_modern(
                    "Air Temp",
                    state.env,
                    "temp",
                    0,
                    80,
                    "°C",
                    "device_thermostat",
                    UIColors.NEON_CYAN,
                )
                refs["gauge_hum"] = circular_gauge_modern(
                    "Humidity",
                    state.env,
                    "humid",
                    0,
                    100,
                    "%",
                    "water_drop",
                    UIColors.NEON_PURPLE,
                )

            ui.separator().classes("bg-slate-700 my-4")
            with ui.row().classes("w-full items-center justify-between px-4"):
                ui.label("PRESSURE").classes("text-sm font-bold text-slate-400")
                ui.label().bind_text_from(
                    state.env, "press", backward=lambda x: f"{x:.0f} hPa"
                ).classes("text-2xl font-bold stat-value").style(f"color: {UIColors.TEXT_MAIN}")
    return refs


def render_status_column(state: RobotState) -> dict:
    """中央カラム（Status & Controls）の構築"""
    refs = {"led_updaters": []}
    with ui.column().classes("col-span-1 gap-6"):
        # System Status Indicators
        card_stat = CyberCard()
        refs["card_stat"] = card_stat
        with card_stat:
            label_header("SYSTEM STATUS", "monitor_heart")
            refs["led_updaters"].append(status_led_modern("Jetson", state.sys_stat, "jetson"))
            refs["led_updaters"].append(
                status_led_modern("Actuator Power", state.sys_stat, "actuator_power")
            )
            refs["led_updaters"].append(
                status_led_modern("Logic Relay", state.sys_stat, "logic_relay")
            )
            refs["led_updaters"].append(status_led_modern("USB Bus", state.sys_stat, "usb_power"))

        # Control Panel
        card_ctrl = CyberCard()
        refs["card_ctrl"] = card_ctrl
        with card_ctrl:
            label_header("System Control", "toggle_on")

            # 各スイッチの表示
            control_switch_modern("JETSON", state.switches, "jetson")
            control_switch_modern("DVL (NC)", state.switches, "dvl")
            control_switch_modern("COM (DVL)", state.switches, "com")
            control_switch_modern("EX1", state.switches, "ex1")
            control_switch_modern("EX8", state.switches, "ex8")
            control_switch_modern("ACTUATOR", state.switches, "actuator")

            # SET Button
            def on_click_set():
                if node_instance:
                    node_instance.publish_switch_command()
                    ui.notify("SEND: System Switch Command", type="info", color=UIColors.NEON_CYAN)
                else:
                    ui.notify("ROS Node Not Connected", type="warning")

            ui.button("SET Switch", on_click=on_click_set).classes(
                "w-full text-lg font-bold tracking-widest h-12 shadow-lg"
            ).props('color="cyan-4" icon="send" no-caps').style(
                f"text-shadow: 0 0 5px rgba(0,0,0,0.5);"
            )

    return refs


def render_nav_column(state: RobotState) -> dict:
    """右カラム（Nav: DVL, IMU, Depth）の構築"""
    refs = {}
    with ui.column().classes("col-span-1 gap-6"):
        # DVL Radar
        card_dvl = CyberCard(no_padding=True)
        refs["card_dvl"] = card_dvl
        with card_dvl:
            with ui.column().classes("p-4 w-full items-center"):
                label_header("DVL (Velocity Vector)", "radar")
                # Radar/Compass UI
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

                # DVL Stats
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

        # IMU
        card_imu = CyberCard()
        refs["card_imu"] = card_imu
        with card_imu:
            with ui.row().classes("items-center justify-between w-full"):
                with ui.column():
                    ui.label("IMU (HEADING)").classes("text-sm font-bold text-slate-500")

                    # Heading
                    ui.label().bind_text_from(
                        state.imu,
                        "orient_z",
                        backward=lambda x: f"{x:3.2f}°",
                    ).classes("text-4xl font-bold stat-value text-orange-400 leading-none")

                    # Temperature
                    with ui.row().classes("items-center gap-2 mt-2"):
                        ui.icon("device_thermostat", size="xs").classes("text-slate-500")
                        ui.label().bind_text_from(
                            state.imu, "temp", backward=lambda x: f"{x:.1f}°C"
                        ).classes("text-lg font-bold stat-value").style(
                            f"color: {UIColors.TEXT_SUB}"
                        )
                ui.button(
                    "Restart",
                    on_click=lambda: node_instance.publish_imu_reset() if node_instance else None,
                ).props('flat color="red" icon="restart_alt"').classes(
                    "border border-red-500/50 text-red-400"
                )

        # Depth Module
        card_depth = CyberCard()
        refs["card_depth"] = card_depth
        with card_depth:
            label_header("Depth", "scuba_diving")
            with ui.row().classes("w-full justify-around"):
                refs["gauge_depth"] = circular_gauge_modern(
                    "Depth",
                    state.depth,
                    "depth",
                    0,
                    10,
                    "m",
                    "vertical_align_bottom",
                    "#3b82f6",
                    "{:.2f}",
                )
                refs["gauge_wtemp"] = circular_gauge_modern(
                    "Water Temp",
                    state.depth,
                    "temp",
                    0,
                    40,
                    "°C",
                    "water",
                    "#06b6d4",
                    "{:.1f}",
                )
    return refs


def render_led_column(state: RobotState) -> dict:
    """4列目（LED Control）の構築"""
    refs = {}
    with ui.column().classes("col-span-1 gap-6"):
        # ==========================================
        # LED Control
        # ==========================================
        card_led = CyberCard()
        refs["card_led"] = card_led
        with card_led:
            label_header("Light Control", "lightbulb")

            # --- Slider Helper Function ---
            def render_led_slider(label, attr_name):
                with ui.column().classes("w-full mb-1 gap-0"):
                    # 1. Main Label & Value
                    with ui.row().classes("w-full justify-between items-end mb-1"):
                        ui.label(label).classes("text-sm font-bold text-slate-400")
                        ui.label().bind_text_from(state, attr_name).classes(
                            "text-lg font-mono font-bold text-cyan-400"
                        )

                    # 2. Scale Labels
                    with ui.row().classes("w-full justify-between px-1"):
                        ui.label("OFF (1100)").classes("text-[10px] font-bold text-slate-600")
                        ui.label("MAX (1900)").classes("text-[10px] font-bold text-slate-600")

                    # 3. Slider (Range: 1100 - 1900)
                    ui.slider(min=1100, max=1900, step=10).bind_value(state, attr_name).props(
                        'color="cyan-4" track-color="grey-8"'
                    ).classes("w-full")

            # Left & Right Sliders
            render_led_slider("LEFT LAMP", "led_left")
            ui.separator().classes("bg-slate-700 my-2")
            render_led_slider("RIGHT LAMP", "led_right")

            # Send Button
            def on_click_led_set():
                if node_instance:
                    node_instance.publish_led_command()
                    ui.notify(
                        f"SENT: LED L:{state.led_left} R:{state.led_right}",
                        type="info",
                        color=UIColors.NEON_CYAN,
                    )
                else:
                    ui.notify("ROS Node Not Connected", type="warning")

            ui.button("SET LIGHTS", on_click=on_click_led_set).classes(
                "w-full text-lg font-bold tracking-widest h-12 shadow-lg mt-4"
            ).props('color="cyan-4" icon="wb_incandescent" no-caps').style(
                f"text-shadow: 0 0 5px rgba(0,0,0,0.5);"
            )

        # ==========================================
        # Camera Tilt Control
        # ==========================================
        card_tilt = CyberCard()
        refs["card_tilt"] = card_tilt
        with card_tilt:
            label_header("Camera Tilt", "videocam")

            with ui.row().classes("w-full items-center justify-between gap-2"):
                # ラベルと入力ボックス
                with ui.row().classes("items-center gap-3"):
                    ui.label("ANGLE").classes("text-xl font-bold stat-value").style(
                        f"color: {UIColors.TEXT_MAIN}"
                    )

                    ui.number(value=0).bind_value(state, "tilt_angle").props(
                        f'outlined dense input-class="text-xl text-center font-mono font-bold" input-style="color: {
                            UIColors.NEON_CYAN
                        }"'
                    ).classes("w-24").style(
                        f"background-color: var(--bg-base); border: 1px solid {UIColors.NEON_CYAN};"
                    )

                    ui.label("deg").classes("text-sm font-bold").style(
                        f"color: {UIColors.TEXT_SUB}"
                    )

                # 送信ボタン
                def on_click_tilt_set():
                    if node_instance:
                        node_instance.publish_tilt_command()
                        ui.notify(
                            f"SENT: Tilt Angle {state.tilt_angle}°",
                            type="info",
                            color=UIColors.NEON_CYAN,
                        )
                    else:
                        ui.notify("ROS Node Not Connected", type="warning")

                ui.button("SET", on_click=on_click_tilt_set).classes(
                    "font-bold tracking-widest h-10 shadow-lg px-6"
                ).props('color="purple-5" icon="publish" no-caps').style(
                    f"text-shadow: 0 0 5px rgba(0,0,0,0.5);"
                )

        # ==========================================
        # Voice Log
        # ==========================================
        card_voice = CyberCard()
        refs["card_voice"] = card_voice
        with card_voice:
            label_header("System Voice", "record_voice_over")

            # max_lines で行数を制限
            log_view = ui.log(max_lines=50).classes(
                "w-full h-32 font-mono text-xs leading-relaxed text-cyan-300 bg-slate-900/50 p-2 rounded border border-slate-700"
            )
            refs["voice_log"] = log_view
    return refs


# ==========================================
# Section 7: Main Entry & Loop
# ==========================================
@ui.page("/")
def index():
    ui.add_head_html(f"<style>{CSS_STYLES}</style>")
    dark_mode = ui.dark_mode(value=True)
    ui.colors(primary=UIColors.NEON_CYAN, secondary=UIColors.BG_BASE, accent=UIColors.NORMAL)

    # 1. Header
    render_header(dark_mode)

    # 2. Main Grid Layout
    main_layout = ui.element("div").classes(
        "w-full min-h-screen p-4 md:p-6 transition-all duration-300"
    )

    with main_layout:
        with ui.grid().classes("grid-cols-1 lg:grid-cols-4 gap-6 w-full max-w-7xl mx-auto"):
            # UIパーツを構築し、動的に更新が必要な要素への参照を受け取る
            refs_col1 = render_power_column(app_state)
            refs_col2 = render_status_column(app_state)
            refs_col3 = render_nav_column(app_state)
            refs_col4 = render_led_column(app_state)

    # 3. Update Loop Logic
    # 複数の辞書に分かれた参照をマージしてアクセスしやすくする
    ui_refs = {**refs_col1, **refs_col2, **refs_col3, **refs_col4}

    for event in app_state.voice_events:
        timestamp = event["time"]
        text = event["text"]
        ui_refs["voice_log"].push(f"[{timestamp}] {text}")
    last_processed_id = app_state.event_counter

    def update_ui():
        nonlocal last_processed_id

        state = app_state
        cfg = state.config

        # Process Voice Queue (音声合成)
        # 自分の last_processed_id より新しいIDを持つイベントを抽出
        new_events = [e for e in state.voice_events if e["id"] > last_processed_id]

        if new_events:
            for event in new_events:
                text = event["text"]
                timestamp = event["time"]

                # 音声再生 (JavaScript)
                escaped_text = text.replace('"', '\\"').replace("'", "\\'")
                js_code = f"""
                    (function() {{
                        const synth = window.speechSynthesis;
                        const text = "{escaped_text}";
                        const uttr = new SpeechSynthesisUtterance(text);
                        
                        function speak() {{
                            const voices = synth.getVoices();
                            let selectedVoice = voices.find(v => v.name.includes('English (Caribbean)+female2'));
                            if (!selectedVoice) {{
                                selectedVoice = voices.find(v => v.name.includes('espeak') && v.name.includes('female'));
                            }}
                            if (!selectedVoice) {{
                                selectedVoice = voices.find(v => v.lang === 'ja-JP' || v.lang === 'ja');
                            }}

                            if (selectedVoice) {{
                                uttr.voice = selectedVoice;
                                uttr.lang = selectedVoice.lang;
                            }} else {{
                                uttr.lang = 'ja-JP';
                            }}
                            
                            uttr.rate = 1.0; 
                            uttr.pitch = 1.0;

                            synth.cancel();
                            synth.speak(uttr);
                        }}

                        if (synth.getVoices().length === 0) {{
                            synth.onvoiceschanged = speak;
                        }} else {{
                            speak();
                        }}
                    }})();
                """
                ui.run_javascript(js_code)

                # ログ表示
                ui_refs["voice_log"].push(f"[{timestamp}] {text}")
                last_processed_id = event["id"]

        # --- A. Gauge Animations ---
        # Note: bind_fromを使っているためテキストは自動更新されるが、プログレスバーの値は明示的に更新が必要
        ui_refs["gauge_air"].value = state.env.temp
        ui_refs["gauge_hum"].value = state.env.humid
        ui_refs["gauge_depth"].value = state.depth.depth
        ui_refs["gauge_wtemp"].value = state.depth.temp

        # --- B. Arrow Animation ---
        angle = math.degrees(math.atan2(state.dvl.vel_y, state.dvl.vel_x))
        scale = min(max(state.dvl.speed_norm / 0.2, 0.2), 1.4)
        ui_refs["arrow_el"].style(
            f"transform: rotate({angle}deg) scale({
                scale
            }); transition: transform 0.2s cubic-bezier(0.4, 0, 0.2, 1);"
        )

        # --- C. LED Indicators ---
        for updater in ui_refs["led_updaters"]:
            updater()

        # --- D. Card Status Borders (Timeout/Error checks) ---
        ui_refs["card_pwr"].update_status(
            state.power.status_id, state.power.is_timeout(cfg.timeout_power)
        )
        ui_refs["card_env"].update_status(
            state.env.status_id, state.env.is_timeout(cfg.timeout_env)
        )

        stat_timeout = state.sys_stat.is_timeout(cfg.timeout_sys_status)
        ui_refs["card_stat"].update_status(state.sys_stat.status_id, stat_timeout)
        ui_refs["card_ctrl"].update_status(0, state.topic_alive_ctrl == 2)

        ui_refs["card_dvl"].update_status(
            state.dvl.status_id, state.dvl.is_timeout(cfg.timeout_dvl)
        )
        ui_refs["card_imu"].update_status(
            state.imu.status_id, state.imu.is_timeout(cfg.timeout_imu)
        )
        ui_refs["card_depth"].update_status(
            state.depth.status_id, state.depth.is_timeout(cfg.timeout_depth)
        )

        ui_refs["card_led"].update_status(0, state.topic_alive_led == 2)
        ui_refs["card_tilt"].update_status(0, state.topic_alive_tilt == 2)

        # --- E. Global Alert ---
        if state.check_global_voltage_warning():
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
    # ROSスレッドの開始
    t = threading.Thread(target=ros_thread, daemon=True)
    t.start()

    # UIの起動
    ui.run(title="KYUBIC SYSTEM", port=8080, reload=False, dark=True, show=False)


if __name__ == "__main__":
    main()
