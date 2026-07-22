"""ROS 2 driver that owns the MAVLink connection to a BlueROV/ArduSub."""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Optional

# pymavlink selects its wire protocol at import time.
os.environ["MAVLINK20"] = "1"

import rclpy
from driver_msgs.msg import IMU, LED, Depth, Gnss, Int32Stamped, PowerState, VehicleState
from geometry_msgs.msg import WrenchStamped
from pymavlink import mavutil
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from .conversions import (
    angle_to_pwm,
    axis_to_pwm,
    euler_to_quaternion,
    radians_to_degrees,
)

RC_RELEASE = 65535
AUX_PWM_MIN = 1100
AUX_PWM_MAX = 1900
MAVLINK_STREAM_RATE_HZ = 10


@dataclass(frozen=True)
class Axis:
    """Mapping and scaling settings for one ArduSub RC axis."""

    channel: int
    limit: float
    invert: bool


class MavlinkDriver(Node):
    """Translate common ROS driver messages to and from MAVLink.

    The node exclusively owns the MAVLink transport for one BlueROV. It sends
    RC overrides and publishes normalized ROS driver messages from telemetry.
    """

    def __init__(self) -> None:
        """Initialize the MAVLink transport, ROS interfaces, and safety timers."""
        super().__init__("mavlink_driver")

        self._endpoint = self.declare_parameter("endpoint", "udpin:0.0.0.0:14550").value
        self._source_system = self.declare_parameter("source_system", 255).value
        self._source_component = self.declare_parameter("source_component", 190).value
        self._command_rate_hz = self.declare_parameter("command_rate_hz", 10.0).value
        self._connection_timeout_s = self.declare_parameter("connection_timeout_s", 3.0).value
        self._reconnect_delay_s = self.declare_parameter("reconnect_delay_s", 2.0).value
        self._command_timeout_s = self.declare_parameter("command_timeout_s", 0.5).value
        self._control_heartbeat_timeout_s = self.declare_parameter(
            "control_heartbeat_timeout_s", 1.0
        ).value
        self._require_control_heartbeat = self.declare_parameter(
            "require_control_heartbeat", True
        ).value
        self._set_armed_timeout_s = self.declare_parameter("set_armed_timeout_s", 3.0).value

        self._pwm_min = self.declare_parameter("pwm_min", 1300).value
        self._pwm_neutral = self.declare_parameter("pwm_neutral", 1500).value
        self._pwm_max = self.declare_parameter("pwm_max", 1700).value
        self._depth_scale = self.declare_parameter("depth_scale", 1.0).value

        self._axes = {
            "roll": self._declare_axis("roll", 2, 3.0),
            "heave": self._declare_axis("heave", 3, 17.0),
            "yaw": self._declare_axis("yaw", 4, 10.0),
            "forward": self._declare_axis("forward", 5, 52.0),
            "lateral": self._declare_axis("lateral", 6, 30.0),
        }
        self._camera_channel = self.declare_parameter("camera_channel", 8).value
        self._camera_angle_min_deg = self.declare_parameter("camera_angle_min_deg", 0.0).value
        self._camera_angle_max_deg = self.declare_parameter("camera_angle_max_deg", 180.0).value
        self._camera_pwm_min = self.declare_parameter("camera_pwm_min", 1100).value
        self._camera_pwm_max = self.declare_parameter("camera_pwm_max", 1900).value
        self._led_left_channel = self.declare_parameter("led_left_channel", 0).value
        self._led_right_channel = self.declare_parameter("led_right_channel", 9).value
        self._validate_configuration()

        self._state_lock = threading.Condition()
        self._connection_lock = threading.Lock()
        self._connection: Optional[mavutil.mavfile] = None
        self._target_system = 0
        self._target_component = 0
        self._connected = False
        self._vehicle_armed = False
        self._custom_mode = 0
        self._system_status = 0
        self._last_vehicle_heartbeat = 0.0
        self._last_control_heartbeat = 0.0
        self._last_wrench = 0.0
        self._last_arm_ack: Optional[tuple[int, float]] = None
        self._stop_event = threading.Event()

        self._axis_values = {name: 0.0 for name in self._axes}
        self._neutral_channels = self._make_neutral_channels()
        self._camera_pwm = self._pwm_neutral
        self._led_left_pwm = AUX_PWM_MIN
        self._led_right_pwm = AUX_PWM_MIN
        self._water_temperature = math.nan

        self._depth_pub = self.create_publisher(Depth, "depth", 10)
        self._imu_pub = self.create_publisher(IMU, "imu", 10)
        self._power_pub = self.create_publisher(PowerState, "power_state", 10)
        self._gnss_pub = self.create_publisher(Gnss, "gnss", 10)
        self._vehicle_state_pub = self.create_publisher(VehicleState, "vehicle_state", 10)

        self.create_subscription(WrenchStamped, "robot_force", self._wrench_callback, 10)
        self.create_subscription(LED, "led", self._led_callback, 10)
        self.create_subscription(Int32Stamped, "camera_tilt", self._camera_callback, 10)
        self.create_subscription(Bool, "heartbeat", self._control_heartbeat_callback, 10)
        self.create_service(SetBool, "set_armed", self._set_armed_callback)

        command_period = 1.0 / max(float(self._command_rate_hz), 1.0)
        self.create_timer(command_period, self._send_override_timer)
        self.create_timer(1.0, self._send_gcs_heartbeat_timer)
        self.create_timer(0.2, self._publish_vehicle_state)

        self._receiver_thread = threading.Thread(
            target=self._receiver_loop, name="blue_rov_mavlink_rx", daemon=True
        )
        self._receiver_thread.start()

    def _declare_axis(self, name: str, default_channel: int, default_limit: float) -> Axis:
        return Axis(
            channel=self.declare_parameter(f"axis.{name}.channel", default_channel).value,
            limit=self.declare_parameter(f"axis.{name}.limit", default_limit).value,
            invert=self.declare_parameter(f"axis.{name}.invert", False).value,
        )

    def _validate_configuration(self) -> None:
        """Validate PWM ranges and prevent conflicting control-channel assignments."""
        if not self._pwm_min <= self._pwm_neutral <= self._pwm_max:
            raise ValueError("pwm_min <= pwm_neutral <= pwm_max is required")
        if self._command_rate_hz <= 0.0:
            raise ValueError("command_rate_hz must be positive")

        axis_channels = [axis.channel for axis in self._axes.values()]
        if len(set(axis_channels)) != len(axis_channels):
            raise ValueError("each control axis must use a unique RC channel")
        if any(channel < 1 or channel > 18 for channel in axis_channels):
            raise ValueError("control axis channels must be in the range 1..18")

        occupied = set(axis_channels)
        if self._camera_channel:
            if not 1 <= self._camera_channel <= 18:
                raise ValueError("camera_channel must be 0 or in the range 1..18")
            if self._camera_channel in occupied:
                raise ValueError("camera_channel conflicts with a control axis")
            occupied.add(self._camera_channel)

        for channel_name, channel in (
            ("led_left_channel", self._led_left_channel),
            ("led_right_channel", self._led_right_channel),
        ):
            if channel < 0 or channel > 18:
                raise ValueError(f"{channel_name} must be 0 or in the range 1..18")
            if channel and channel in occupied:
                raise ValueError(f"{channel_name} conflicts with a control or camera channel")

    def _make_neutral_channels(self) -> list[int]:
        """Create the fixed portion of an RC override packet."""
        channels = [RC_RELEASE] * 18
        for axis in self._axes.values():
            channels[axis.channel - 1] = self._pwm_neutral
        return channels

    def _wrench_callback(self, message: WrenchStamped) -> None:
        with self._state_lock:
            self._axis_values["forward"] = message.wrench.force.x
            self._axis_values["lateral"] = message.wrench.force.y
            self._axis_values["heave"] = message.wrench.force.z
            self._axis_values["roll"] = message.wrench.torque.x
            self._axis_values["yaw"] = message.wrench.torque.z
            self._last_wrench = time.monotonic()

    def _led_callback(self, message: LED) -> None:
        with self._state_lock:
            self._led_left_pwm = self._clip_aux_pwm(message.left)
            self._led_right_pwm = self._clip_aux_pwm(message.right)

    def _camera_callback(self, message: Int32Stamped) -> None:
        with self._state_lock:
            self._camera_pwm = angle_to_pwm(
                message.data,
                self._camera_angle_min_deg,
                self._camera_angle_max_deg,
                self._camera_pwm_min,
                self._camera_pwm_max,
            )

    def _control_heartbeat_callback(self, message: Bool) -> None:
        if message.data:
            with self._state_lock:
                self._last_control_heartbeat = time.monotonic()

    def _clip_aux_pwm(self, value: int) -> int:
        return max(AUX_PWM_MIN, min(AUX_PWM_MAX, int(value)))

    def _receiver_loop(self) -> None:
        while not self._stop_event.is_set():
            connection = None
            try:
                connection_started = time.monotonic()
                connection = mavutil.mavlink_connection(
                    self._endpoint,
                    source_system=self._source_system,
                    source_component=self._source_component,
                )
                with self._connection_lock:
                    self._connection = connection
                self.get_logger().info(f"Waiting for MAVLink heartbeat on {self._endpoint}")

                while not self._stop_event.is_set():
                    message = connection.recv_match(blocking=True, timeout=0.5)
                    if message is not None:
                        self._handle_mavlink_message(message)

                    with self._state_lock:
                        last_heartbeat = self._last_vehicle_heartbeat
                    now = time.monotonic()
                    heartbeat_age = now - last_heartbeat
                    if last_heartbeat and heartbeat_age > self._connection_timeout_s:
                        raise ConnectionError("vehicle heartbeat timed out")
                    if not last_heartbeat and now - connection_started > self._connection_timeout_s:
                        raise ConnectionError("timed out waiting for vehicle heartbeat")
            except Exception as error:  # Connection errors must not terminate the driver.
                if not self._stop_event.is_set():
                    self.get_logger().warning(f"MAVLink connection lost: {error}")
                self._set_disconnected()
                self._stop_event.wait(float(self._reconnect_delay_s))
            finally:
                with self._connection_lock:
                    if self._connection is connection:
                        self._connection = None
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass

    def _handle_mavlink_message(self, message) -> None:
        message_type = message.get_type()
        if message_type == "BAD_DATA":
            return

        if message_type == "HEARTBEAT":
            if message.type != mavutil.mavlink.MAV_TYPE_SUBMARINE:
                return
            source_system = message.get_srcSystem()
            source_component = message.get_srcComponent()
            with self._state_lock:
                if self._connected and source_system != self._target_system:
                    return
                newly_connected = not self._connected
                self._target_system = source_system
                self._target_component = source_component
                self._connected = True
                self._vehicle_armed = bool(
                    message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                self._custom_mode = message.custom_mode
                self._system_status = message.system_status
                self._last_vehicle_heartbeat = time.monotonic()
                self._state_lock.notify_all()
            if newly_connected:
                self._request_data_stream(source_system, source_component)
            return

        with self._state_lock:
            if not self._connected or message.get_srcSystem() != self._target_system:
                return

        if message_type == "BATTERY_STATUS":
            self._publish_power_state(message)
        elif message_type == "VFR_HUD":
            self._publish_depth(message.alt * self._depth_scale)
        elif message_type == "SCALED_PRESSURE2":
            with self._state_lock:
                self._water_temperature = message.temperature / 100.0
        elif message_type == "ATTITUDE":
            self._publish_imu(message)
        elif message_type == "GPS_RAW_INT":
            self._publish_gnss(message)
        elif message_type == "COMMAND_ACK":
            self._handle_command_ack(message)

    def _publish_depth(self, depth: float) -> None:
        with self._state_lock:
            temperature = self._water_temperature
        message = Depth()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "depth"
        message.status.id = 0
        message.watertype = Depth.SEA_WATER
        message.depth = float(depth)
        message.temperature = float(temperature)
        self._depth_pub.publish(message)

    def _publish_imu(self, mav_message) -> None:
        message = IMU()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "imu"
        message.status.id = 0
        message.temperature = math.nan
        message.accel.x = math.nan
        message.accel.y = math.nan
        message.accel.z = math.nan
        message.gyro.x = radians_to_degrees(mav_message.rollspeed)
        message.gyro.y = radians_to_degrees(mav_message.pitchspeed)
        message.gyro.z = radians_to_degrees(mav_message.yawspeed)
        message.orient.x = radians_to_degrees(mav_message.roll)
        message.orient.y = radians_to_degrees(mav_message.pitch)
        message.orient.z = radians_to_degrees(mav_message.yaw)
        qx, qy, qz, qw = euler_to_quaternion(mav_message.roll, mav_message.pitch, mav_message.yaw)
        message.qtn.x = qx
        message.qtn.y = qy
        message.qtn.z = qz
        message.qtn.w = qw
        self._imu_pub.publish(message)

    def _publish_power_state(self, mav_message) -> None:
        message = PowerState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status.id = 0
        voltage = (
            mav_message.voltages[0] / 1000.0
            if mav_message.voltages and mav_message.voltages[0] != 65535
            else math.nan
        )
        current = (
            mav_message.current_battery / 100.0 if mav_message.current_battery != -1 else math.nan
        )
        message.log_voltage = math.nan
        message.log_current = math.nan
        message.log_power = math.nan
        message.log_temp = math.nan
        message.act_voltage = voltage
        message.act_current = current
        message.act_power = voltage * current if math.isfinite(voltage * current) else math.nan
        message.act_temp = math.nan
        self._power_pub.publish(message)

    def _publish_gnss(self, mav_message) -> None:
        message = Gnss()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "gps"
        message.status.id = 0 if mav_message.fix_type >= 3 else 1
        message.fix.header = message.header
        message.fix.status.status = (
            NavSatStatus.STATUS_FIX if mav_message.fix_type >= 3 else NavSatStatus.STATUS_NO_FIX
        )
        message.fix.status.service = NavSatStatus.SERVICE_GPS
        message.fix.latitude = mav_message.lat / 1e7
        message.fix.longitude = mav_message.lon / 1e7
        message.fix.altitude = mav_message.alt / 1000.0
        message.azimuth = math.nan
        message.geoid_altitude = math.nan
        message.geoid_separation = math.nan
        message.snr = []
        self._gnss_pub.publish(message)

    def _handle_command_ack(self, message) -> None:
        if message.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            return
        with self._state_lock:
            self._last_arm_ack = (message.result, time.monotonic())
            self._state_lock.notify_all()

    def _request_data_stream(self, target_system: int, target_component: int) -> None:
        """Request the telemetry stream once after establishing a vehicle connection."""
        with self._connection_lock:
            connection = self._connection
            if connection is None:
                return
            try:
                connection.target_system = target_system
                connection.target_component = target_component
                connection.mav.request_data_stream_send(
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    MAVLINK_STREAM_RATE_HZ,
                    1,
                )
            except Exception as error:
                self.get_logger().warning(f"Failed to request MAVLink data stream: {error}")

    def _send_override_timer(self) -> None:
        with self._state_lock:
            now = time.monotonic()
            control_alive = (
                not self._require_control_heartbeat
                or now - self._last_control_heartbeat <= self._control_heartbeat_timeout_s
            )
            command_alive = now - self._last_wrench <= self._command_timeout_s
            active = self._connected and self._vehicle_armed and control_alive and command_alive
            values = dict(self._axis_values)
            camera_pwm = self._camera_pwm
            led_left_pwm = self._led_left_pwm
            led_right_pwm = self._led_right_pwm
            target_system = self._target_system
            target_component = self._target_component

        channels = self._build_rc_channels(active, values)
        self._set_channel(channels, self._camera_channel, camera_pwm)
        if self._led_left_channel and self._led_left_channel == self._led_right_channel:
            self._set_channel(channels, self._led_left_channel, max(led_left_pwm, led_right_pwm))
        else:
            self._set_channel(channels, self._led_left_channel, led_left_pwm)
            self._set_channel(channels, self._led_right_channel, led_right_pwm)
        self._send_rc_override(channels, target_system, target_component)

    def _build_rc_channels(self, active: bool, axis_values: dict[str, float]) -> list[int]:
        """Build an RC override packet from a stable command snapshot."""
        channels = self._neutral_channels.copy()
        if not active:
            return channels
        for name, axis in self._axes.items():
            channels[axis.channel - 1] = axis_to_pwm(
                axis_values[name],
                axis.limit,
                axis.invert,
                self._pwm_min,
                self._pwm_neutral,
                self._pwm_max,
            )
        return channels

    def _set_channel(self, channels: list[int], channel: int, value: int) -> None:
        if 1 <= int(channel) <= len(channels):
            channels[int(channel) - 1] = int(value)

    def _send_gcs_heartbeat_timer(self) -> None:
        with self._connection_lock:
            connection = self._connection
            if connection is None:
                return
            try:
                connection.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )
            except Exception as error:
                self.get_logger().warning(f"Failed to send GCS heartbeat: {error}")

    def _send_rc_override(
        self, channels: list[int], target_system: int, target_component: int
    ) -> None:
        if not target_system:
            return
        with self._connection_lock:
            connection = self._connection
            if connection is None:
                return
            try:
                message = connection.mav.rc_channels_override_encode(
                    target_system, target_component, *channels
                )
                connection.mav.send(message)
            except Exception as error:
                self.get_logger().warning(f"Failed to send RC override: {error}")

    def _set_armed_callback(self, request: SetBool.Request, response: SetBool.Response):
        with self._state_lock:
            if not self._connected or not self._target_system:
                response.success = False
                response.message = "BlueROV is not connected"
                return response
            self._last_arm_ack = None
            target_system = self._target_system
            target_component = self._target_component

        with self._connection_lock:
            connection = self._connection
            if connection is None:
                response.success = False
                response.message = "MAVLink connection is unavailable"
                return response
            try:
                connection.mav.command_long_send(
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1.0 if request.data else 0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
            except Exception as error:
                response.success = False
                response.message = f"Failed to send arm command: {error}"
                return response

        deadline = time.monotonic() + float(self._set_armed_timeout_s)
        with self._state_lock:
            while time.monotonic() < deadline:
                ack_accepted = self._last_arm_ack is not None and self._last_arm_ack[0] == (
                    mavutil.mavlink.MAV_RESULT_ACCEPTED
                )
                if ack_accepted and self._vehicle_armed == request.data:
                    response.success = True
                    response.message = "Vehicle arm state confirmed by MAVLink heartbeat"
                    return response
                self._state_lock.wait(timeout=0.1)

        response.success = False
        response.message = "Arm command was not acknowledged and confirmed before timeout"
        return response

    def _publish_vehicle_state(self) -> None:
        with self._state_lock:
            connected = self._connected and (
                time.monotonic() - self._last_vehicle_heartbeat <= self._connection_timeout_s
            )
            message = VehicleState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.status.id = 0 if connected else 2
            message.connected = connected
            message.armed = self._vehicle_armed if connected else False
            message.target_system = self._target_system
            message.target_component = self._target_component
            message.custom_mode = self._custom_mode
            message.system_status = self._system_status
        self._vehicle_state_pub.publish(message)

    def _set_disconnected(self) -> None:
        with self._state_lock:
            self._connected = False
            self._vehicle_armed = False
            self._target_system = 0
            self._target_component = 0
            self._last_vehicle_heartbeat = 0.0
            self._state_lock.notify_all()

    def destroy_node(self) -> bool:
        """Send neutral axis commands and stop the MAVLink receiver before shutdown.

        Returns:
            bool: The result reported by the base ROS node implementation.
        """
        with self._state_lock:
            target_system = self._target_system
            target_component = self._target_component
        self._send_rc_override(self._neutral_channels.copy(), target_system, target_component)
        self._stop_event.set()
        self._receiver_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None) -> None:
    """Run the BlueROV MAVLink driver ROS 2 node.

    Args:
        args: Optional ROS command-line arguments.
    """
    rclpy.init(args=args)
    node = MavlinkDriver()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
