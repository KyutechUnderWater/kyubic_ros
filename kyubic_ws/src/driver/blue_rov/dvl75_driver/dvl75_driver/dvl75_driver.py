"""ROS 2 UDP driver for the Cerulean Sonar DVL-75."""

from __future__ import annotations

import socket
import threading
import time
from typing import Final

import rclpy
from blue_rov_msgs.msg import DVL75
from builtin_interfaces.msg import Time
from common_msgs.msg import Status
from driver_msgs.msg import DVL
from driver_msgs.srv import Command
from rclpy.node import Node

from .parser import Dvext, Dvl75ParseError, Dvpdl, parse_sentence

DEFAULT_DVL_ADDRESS: Final = "192.168.2.3"
DEFAULT_DVL_PORT: Final = 50000


class Dvl75Driver(Node):
    """Publish common DVL measurements from Cerulean DVL-75 UDP messages."""

    def __init__(self) -> None:
        """Initialize ROS interfaces, UDP reception, and optional DVL setup."""
        super().__init__("dvl75_driver")

        # Network endpoints
        self._dvl_address = self.declare_parameter("dvl_address", DEFAULT_DVL_ADDRESS).value
        self._dvl_command_port = self.declare_parameter("dvl_command_port", DEFAULT_DVL_PORT).value
        self._bind_address = self.declare_parameter("bind_address", "0.0.0.0").value
        self._listen_port = self.declare_parameter("listen_port", 27000).value

        # ROS output and DVL measurement validation
        self._frame_id = self.declare_parameter("frame_id", "dvl75_link").value
        self._timeout_s = self.declare_parameter("timeout_s", 1.0).value
        self._dvext_timeout_s = self.declare_parameter("dvext_timeout_s", 1.0).value
        self._validate_checksum = self.declare_parameter("validate_checksum", True).value
        self._minimum_confidence = self.declare_parameter("minimum_confidence", 50).value
        self._minimum_locked_beams = self.declare_parameter("minimum_locked_beams", 3).value

        # DVL vehicle-frame conversion
        self._position_axis_sign = tuple(
            self.declare_parameter("position_axis_sign", [1.0, 1.0, 1.0]).value
        )
        self._angle_axis_sign = tuple(
            self.declare_parameter("angle_axis_sign", [1.0, 1.0, 1.0]).value
        )

        # Explicit persistent DVL configuration
        self._apply_device_config = self.declare_parameter("apply_device_config", False).value
        self._device_host_address = self.declare_parameter("device_host_address", "").value
        self._startup_commands = self._normalize_startup_commands(
            self.declare_parameter("startup_commands", []).value
        )
        self._validate_configuration()

        self._socket_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._last_dvext: Dvext | None = None
        self._last_dvext_time = 0.0
        self._last_dvpdl_time = time.monotonic()
        self._timeout_reported = False
        self._last_tracking_status: int | None = None
        self._dvpdl_enable_requested_at: float | None = None

        self._dvl_pub = self.create_publisher(DVL, "dvl", 10)
        self._dvl75_pub = self.create_publisher(DVL75, "dvl75", 10)
        self.create_service(Command, "command", self._command_callback)
        self.create_timer(0.2, self._timeout_callback)

        self._socket = self._create_socket()
        self._log_connection_configuration()
        if self._apply_device_config:
            self._apply_configuration()
        else:
            self.get_logger().info(
                "DVL persistent configuration is disabled; expecting DVEXT and DVPDL output."
            )

        self._receiver_thread = threading.Thread(
            target=self._receive_loop, name="dvl75_udp_rx", daemon=True
        )
        self._receiver_thread.start()

    def _validate_configuration(self) -> None:
        """Validate parameters that affect packet acceptance and coordinate conversion."""
        if not 1 <= self._listen_port <= 65535:
            raise ValueError("listen_port must be in the range 1..65535")
        if not 1 <= self._dvl_command_port <= 65535:
            raise ValueError("dvl_command_port must be in the range 1..65535")
        if self._timeout_s <= 0.0 or self._dvext_timeout_s <= 0.0:
            raise ValueError("timeout_s and dvext_timeout_s must be positive")
        if not 0 <= self._minimum_confidence <= 100:
            raise ValueError("minimum_confidence must be in the range 0..100")
        if not 3 <= self._minimum_locked_beams <= 4:
            raise ValueError("minimum_locked_beams must be in the range 3..4")
        self._validate_axis_sign("position_axis_sign", self._position_axis_sign)
        self._validate_axis_sign("angle_axis_sign", self._angle_axis_sign)

    @staticmethod
    def _normalize_startup_commands(value: object) -> tuple[str, ...]:
        """Normalize an optional ROS string-array parameter to a tuple.

        ROS parameter YAML represents an empty array without an element type.
        Depending on the ROS distribution, rclpy can therefore return ``None``.

        Args:
            value: Declared ROS parameter value.

        Returns:
            Tuple of newline-free command candidates. ``None`` becomes an empty tuple.

        Raises:
            ValueError: If the value is not an iterable of strings.
        """
        if value is None:
            return ()
        if isinstance(value, (str, bytes)):
            raise ValueError("startup_commands must be a string array")
        try:
            commands = tuple(value)
        except TypeError as error:
            raise ValueError("startup_commands must be a string array") from error
        if not all(isinstance(command, str) for command in commands):
            raise ValueError("startup_commands must be a string array")
        return commands

    @staticmethod
    def _validate_axis_sign(name: str, values: tuple[float, ...]) -> None:
        """Ensure an axis-sign parameter has exactly three non-zero values.

        Args:
            name: Parameter name used in the validation error.
            values: Candidate signs for the X, Y, and Z axes.

        Raises:
            ValueError: If the value count is not three or any sign is zero.
        """
        if len(values) != 3 or any(float(value) == 0.0 for value in values):
            raise ValueError(f"{name} must contain three non-zero values")

    def _create_socket(self) -> socket.socket:
        """Create a UDP socket that receives DVL broadcasts and unicast packets.

        Returns:
            Bound UDP socket configured for DVL traffic.
        """
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        udp_socket.settimeout(0.2)
        udp_socket.bind((self._bind_address, self._listen_port))
        self.get_logger().info(
            f"Listening for DVL-75 packets on {self._bind_address}:{self._listen_port}"
        )
        return udp_socket

    def _log_connection_configuration(self) -> None:
        """Log all network endpoints before any DVL command is sent."""
        self.get_logger().info(
            "DVL-75 endpoints: "
            f"receive={self._bind_address}:{self._listen_port}, "
            f"dvl_address={self._dvl_address}, "
            f"command_port={self._dvl_command_port}, "
            f"apply_device_config={self._apply_device_config}"
        )

    def _apply_configuration(self) -> None:
        """Apply required and user-provided persistent DVL settings over UDP.

        Raises:
            OSError: If a UDP command cannot be sent.
            ValueError: If an additional startup command is invalid or disables required output.
        """
        commands: list[str] = []
        if self._device_host_address:
            commands.append(f"HOST-ADDRESS {self._device_host_address}")
        commands.append("SEND-DVEXT ON")
        commands.append("SEND-DVPDL ON")
        commands.extend(str(command) for command in self._startup_commands)

        for command in commands:
            self._send_command(command)
        self.get_logger().warning(
            f"Applied {len(commands)} persistent DVL configuration command(s)."
        )

    def _command_callback(
        self, request: Command.Request, response: Command.Response
    ) -> Command.Response:
        """Forward one explicit ASCII DVL command to the configured DVL address.

        Args:
            request: Service request containing one DVL command line.
            response: Service response populated with the send result.

        Returns:
            Response describing whether the command was sent or rejected.
        """
        try:
            self._send_command(request.command)
        except ValueError as error:
            response.output = f"Command rejected: {error}"
        except OSError as error:
            response.output = f"Command send failed: {error}"
        else:
            response.output = "Command sent; inspect DVL output for any device response."
        return response

    def _send_command(self, command: str) -> None:
        """Send a newline-terminated DVL command without waiting for a response.

        Args:
            command: One ASCII DVL command without a line terminator.

        Raises:
            OSError: If the UDP socket cannot send the command.
            ValueError: If the command is empty, multi-line, or disables required output.
        """
        normalized = command.strip()
        if not normalized:
            raise ValueError("command must not be empty")
        if "\n" in normalized or "\r" in normalized:
            raise ValueError("command must be a single line")
        if tuple(normalized.upper().split()) in (
            ("SEND-DVEXT", "OFF"),
            ("SEND-DVPDL", "OFF"),
        ):
            raise ValueError("DVEXT and DVPDL output are required and cannot be disabled")
        with self._socket_lock:
            self._socket.sendto(
                f"{normalized}\n".encode("ascii"),
                (self._dvl_address, self._dvl_command_port),
            )
        if normalized.upper() == "SEND-DVPDL ON":
            self._dvpdl_enable_requested_at = time.monotonic()
        self.get_logger().info(f"Sent DVL command: {normalized}")

    def _receive_loop(self) -> None:
        """Receive, validate, and dispatch DVL UDP packets until shutdown."""
        while not self._stop_event.is_set():
            try:
                packet, sender = self._socket.recvfrom(4096)
            except TimeoutError:
                continue
            except OSError as error:
                if not self._stop_event.is_set():
                    self.get_logger().error(f"DVL UDP receive failed: {error}")
                return

            received_at = time.monotonic()
            received_stamp = self.get_clock().now().to_msg()
            if sender[0] != self._dvl_address:
                self.get_logger().debug(
                    f"Ignoring packet from {sender[0]}; expected DVL at {self._dvl_address}"
                )
                continue
            self._process_packet(packet, received_at, received_stamp)

    def _process_packet(self, packet: bytes, received_at: float, received_stamp: Time) -> None:
        """Parse one UDP payload using its reception timestamps.

        Args:
            packet: Raw UDP payload received from the configured DVL.
            received_at: Monotonic host reception time for freshness and timeout checks.
            received_stamp: ROS Clock reception time used as the message header stamp.
        """
        for sentence in packet.splitlines():
            if not sentence:
                continue
            try:
                measurement = parse_sentence(sentence, self._validate_checksum)
            except Dvl75ParseError as error:
                self.get_logger().warning(f"Discarded invalid DVL packet: {error}")
                self._publish_error_dvl(received_stamp)
                continue

            if isinstance(measurement, Dvext):
                self._last_dvext = measurement
                self._last_dvext_time = received_at
            elif isinstance(measurement, Dvpdl):
                self._last_dvpdl_time = received_at
                self._timeout_reported = False
                self._dvpdl_enable_requested_at = None
                dvext = self._get_recent_dvext(received_at)
                position_delta = self._apply_axis_sign(
                    measurement.position_delta, self._position_axis_sign
                )
                angle_delta = self._apply_axis_sign(measurement.angle_delta, self._angle_axis_sign)
                self._publish_dvl75(dvext, measurement, position_delta, angle_delta, received_stamp)
                self._publish_dvpdl(measurement, dvext, position_delta, received_stamp)

    def _get_recent_dvext(self, received_at: float) -> Dvext | None:
        """Return the latest DVEXT measurement when it is still fresh.

        Args:
            received_at: Monotonic host time of the current DVPDL packet.

        Returns:
            Latest DVEXT measurement within ``dvext_timeout_s``; otherwise ``None``.
        """
        if received_at - self._last_dvext_time <= self._dvext_timeout_s:
            return self._last_dvext
        return None

    def _publish_dvpdl(
        self,
        dvpdl: Dvpdl,
        dvext: Dvext | None,
        position_delta: tuple[float, float, float],
        received_stamp: Time,
    ) -> None:
        """Convert one DVPDL measurement into the shared DVL ROS message.

        Args:
            dvpdl: Parsed incremental DVL measurement.
            dvext: Fresh extended DVL measurement, if available.
            position_delta: Position delta after configured axis-sign conversion.
            received_stamp: ROS Clock time assigned to the output header.
        """
        message = DVL()
        message.header.stamp = received_stamp
        message.header.frame_id = self._frame_id

        locked_beam_count = sum(dvext.beam_lock) if dvext is not None else 0
        measurement_valid = (
            dvext is not None
            and dvext.bottom_lock
            and locked_beam_count >= self._minimum_locked_beams
            and dvpdl.confidence >= self._minimum_confidence
            and dvpdl.delta_time_us > 0
        )
        message.velocity_valid = measurement_valid
        if measurement_valid:
            delta_seconds = dvpdl.delta_time_us / 1_000_000.0
            self._set_vector(
                message.velocity,
                tuple(component / delta_seconds for component in position_delta),
            )
            message.altitude = dvext.altitude
            message.status.id = Status.NORMAL if locked_beam_count == 4 else Status.WARNING
        else:
            message.status.id = Status.ERROR

        self._report_tracking_status(message.status.id, locked_beam_count, dvpdl.confidence)

        self._dvl_pub.publish(message)

    def _publish_dvl75(
        self,
        dvext: Dvext | None,
        dvpdl: Dvpdl,
        position_delta: tuple[float, float, float],
        angle_delta: tuple[float, float, float],
        received_stamp: Time,
    ) -> None:
        """Publish one DVL-75 snapshot for each received DVPDL sentence.

        Args:
            dvext: Fresh extended DVL measurement, if available.
            dvpdl: Parsed incremental DVL measurement.
            position_delta: Position delta after configured axis-sign conversion.
            angle_delta: Angle delta after configured axis-sign conversion.
            received_stamp: ROS Clock time assigned to the output header.
        """
        message = DVL75()
        message.header.stamp = received_stamp
        message.header.frame_id = self._frame_id

        if dvext is not None:
            message.dvext_valid = True
            message.bottom_lock = dvext.bottom_lock
            message.data_skips = dvext.data_skips
            message.velocity_up = dvext.velocity_up
            message.altitude = dvext.altitude
            message.velocity_north = dvext.velocity_north
            message.velocity_east = dvext.velocity_east
            message.beam_lock = list(dvext.beam_lock)
            message.beam_velocity = list(dvext.beam_velocity)
            message.beam_range = list(dvext.beam_range)

        message.dvpdl_valid = True
        message.device_time_us = dvpdl.device_time_us
        message.delta_time_us = dvpdl.delta_time_us
        self._set_vector(message.angle_delta, angle_delta)
        self._set_vector(message.position_delta, position_delta)
        message.confidence = dvpdl.confidence

        self._dvl75_pub.publish(message)

    def _publish_error_dvl(self, stamp: Time) -> None:
        """Publish a shared DVL error state without a valid velocity measurement.

        Args:
            stamp: ROS Clock time assigned to the error message header.
        """
        message = DVL()
        message.header.stamp = stamp
        message.header.frame_id = self._frame_id
        message.velocity_valid = False
        message.status.id = Status.ERROR
        self._dvl_pub.publish(message)

    def _report_tracking_status(self, status: int, locked_beam_count: int, confidence: int) -> None:
        """Log a DVL measurement-quality transition without flooding the console.

        Args:
            status: ``common_msgs/Status`` identifier for the current measurement.
            locked_beam_count: Number of bottom-locked DVL beams.
            confidence: DVL confidence value in the range 0 to 100.
        """
        if status == self._last_tracking_status:
            return

        self._last_tracking_status = status
        detail = (
            f"DVL-75 tracking status={status}, locked_beams={locked_beam_count}, "
            f"confidence={confidence}"
        )
        if status == Status.ERROR:
            self.get_logger().error(detail)
        elif status == Status.WARNING:
            self.get_logger().warning(detail)
        else:
            self.get_logger().info(detail)

    @staticmethod
    def _set_vector(target: object, values: tuple[float, float, float]) -> None:
        """Copy a three-element tuple into a ROS Vector3-like object.

        Args:
            target: Object exposing writable ``x``, ``y``, and ``z`` attributes.
            values: Values assigned to the three vector axes.
        """
        target.x, target.y, target.z = values

    @staticmethod
    def _apply_axis_sign(
        values: tuple[float, float, float], signs: tuple[float, ...]
    ) -> tuple[float, float, float]:
        """Apply configurable signs to a DVL vehicle-frame vector.

        Args:
            values: Source vector in DVL coordinates.
            signs: Non-zero configured signs for the X, Y, and Z axes.

        Returns:
            Vector with each component multiplied by its configured sign.
        """
        return (
            values[0] * float(signs[0]),
            values[1] * float(signs[1]),
            values[2] * float(signs[2]),
        )

    def _timeout_callback(self) -> None:
        """Publish one error state when no incremental DVL data arrives in time."""
        if time.monotonic() - self._last_dvpdl_time <= self._timeout_s or self._timeout_reported:
            return

        self._publish_error_dvl(self.get_clock().now().to_msg())
        self._timeout_reported = True
        if self._dvpdl_enable_requested_at is not None:
            elapsed = time.monotonic() - self._dvpdl_enable_requested_at
            self.get_logger().error(
                "DVL-75 configuration error: no DVPDL received after "
                f"SEND-DVPDL ON ({elapsed:.2f} s, target={self._dvl_address}:"
                f"{self._dvl_command_port}). UDP send success does not confirm DVL receipt."
            )
        else:
            self.get_logger().error(
                f"DVL-75 timeout: no DVPDL sentence received within {self._timeout_s:.2f} s"
            )

    def destroy_node(self) -> bool:
        """Stop the receiver thread and release its UDP socket.

        Returns:
            Result of the base ROS node destruction.
        """
        self._stop_event.set()
        self._socket.close()
        self._receiver_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the DVL-75 ROS node.

    Args:
        args: Optional ROS argument vector passed to ``rclpy.init``.
    """
    rclpy.init(args=args)
    node = Dvl75Driver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
