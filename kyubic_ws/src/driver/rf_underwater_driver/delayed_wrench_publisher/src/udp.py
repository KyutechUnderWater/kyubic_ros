import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import socket
import threading
import time

# Custom message import
# 実行には planner_msgs パッケージのビルドが必要です
try:
    from planner_msgs.msg import WrenchPlan
except ImportError:
    print("Error: Could not import planner_msgs. Make sure the package is built and sourced.")
    # For coding assistance, we assume the class exists or use a dummy if testing without it
    # sys.exit(1)


class UdpDelayedWrenchPublisher(Node):
    """Listens for UDP packets and triggers a delayed WrenchPlan publication sequence."""

    def __init__(self):
        super().__init__("udp_delayed_wrench_publisher")

        # --- Declare Parameters (Default values from C++ code) ---
        self.declare_parameter("priority", 1)
        self.declare_parameter("timeout_ms", 1000)
        self.declare_parameter("initial_delay_sec", 10.0)
        self.declare_parameter("execution_duration_sec", 15.0)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("target_x", 1.0)
        self.declare_parameter("target_y", 1.0)
        self.declare_parameter("target_z", 0.3)
        self.declare_parameter("udp_port", 50000)

        # --- Get Parameters ---
        self.priority = self.get_parameter("priority").value
        self.timeout_ms = self.get_parameter("timeout_ms").value
        self.initial_delay_sec = self.get_parameter("initial_delay_sec").value
        self.execution_duration_sec = self.get_parameter("execution_duration_sec").value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.target_x = self.get_parameter("target_x").value
        self.target_y = self.get_parameter("target_y").value
        self.target_z = self.get_parameter("target_z").value
        self.udp_port = self.get_parameter("udp_port").value

        # --- State Flags ---
        self.is_triggered = False
        self.is_publishing = False

        # --- Publisher ---
        self.publisher_ = self.create_publisher(
            WrenchPlan, "/planner/wrench_planner/wrench_plan", 10
        )

        # --- Timers (Initially None or canceled behavior via logic) ---
        self.delay_timer = None
        self.stop_timer = None
        self.publish_timer = None

        # --- Initialize Message ---
        self.wrench_msg = WrenchPlan()
        self._init_wrench_plan_data()

        # --- Start UDP Listener Thread ---
        self.udp_thread = threading.Thread(target=self._udp_listener, daemon=True)
        self.udp_thread.start()

        self.get_logger().info(f"Ready. Waiting for UDP trigger on port {self.udp_port}...")

    def _init_wrench_plan_data(self) -> None:
        """Initializes the WrenchPlan message fields."""
        # Priority
        self.wrench_msg.priority.id = int(self.priority)
        self.wrench_msg.priority.timeout_ms = int(self.timeout_ms)

        # Flags
        self.wrench_msg.has_master = False
        self.wrench_msg.has_slave = False

        # Z Mode (Using constant from class if available, else assuming 1 or specific value)
        # Assuming Z_MODE_DEPTH is available in the Python binding
        if hasattr(WrenchPlan, "Z_MODE_DEPTH"):
            self.wrench_msg.z_mode = WrenchPlan.Z_MODE_DEPTH
        else:
            self.wrench_msg.z_mode = 1  # Fallback if constant not found

        # Targets
        self.wrench_msg.targets.x = float(self.target_x)
        self.wrench_msg.targets.y = float(self.target_y)
        self.wrench_msg.targets.z = float(self.target_z)

    def _udp_listener(self) -> None:
        """Runs in a background thread to listen for UDP packets."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", self.udp_port))

        while rclpy.ok():
            try:
                data, addr = sock.recvfrom(1024)
                msg = data.decode("utf-8").strip()
                self.get_logger().info(f"UDP Received from {addr}: {msg}")

                # Trigger the sequence
                # Using add_on_set_parameters_callback or similar isn't needed for simple trigger
                # We call the trigger method. Since timers are created in main loop context via
                # rclpy spinning, we need to be careful. Ideally, use a guard condition or
                # just schedule a callback. Here we call a method that creates timers.
                self.trigger_sequence()

            except Exception as e:
                self.get_logger().error(f"UDP Error: {e}")

    def trigger_sequence(self) -> None:
        """Starts the initial delay timer."""
        if self.is_triggered:
            self.get_logger().warn("Sequence already triggered. Ignoring.")
            return

        self.is_triggered = True
        self.get_logger().info(f"Triggered! Starting {self.initial_delay_sec}s delay...")

        # Start Delay Timer (One-shot)
        # Note: In rclpy, creating timers from another thread can be risky.
        # But commonly it works if the executor handles it.
        # Safest way in mixed-thread: use a reentrant callback group or logic flag.
        # Here we rely on Python's GIL + rclpy internals handling it.
        self.delay_timer = self.create_timer(self.initial_delay_sec, self._on_delay_complete)

    def _on_delay_complete(self) -> None:
        """Callback when initial delay finishes. Starts publishing."""
        self.get_logger().info(f"{self.initial_delay_sec}s passed.")
        self.get_logger().info(
            f"Starting {self.publish_rate_hz}Hz publication for {self.execution_duration_sec}s. "
            f"Target: {self.target_x}, {self.target_y}, {self.target_z}"
        )

        # Cancel delay timer
        if self.delay_timer:
            self.delay_timer.cancel()
            self.delay_timer = None

        # Start Publication Timer
        period = 1.0 / self.publish_rate_hz
        self.publish_timer = self.create_timer(period, self._publish_callback)

        # Start Stop Timer
        self.stop_timer = self.create_timer(self.execution_duration_sec, self._on_stop_sequence)
        self.is_publishing = True

    def _publish_callback(self) -> None:
        """Publishes the WrenchPlan message."""
        if not self.is_publishing:
            return

        # Update timestamp
        self.wrench_msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(self.wrench_msg)

    def _on_stop_sequence(self) -> None:
        """Callback to stop publishing after duration."""
        self.get_logger().info(f"{self.execution_duration_sec}s duration passed. Stopping.")

        self.is_publishing = False

        # Cancel timers
        if self.publish_timer:
            self.publish_timer.cancel()
            self.publish_timer = None

        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None

        # Reset trigger state if you want to allow re-triggering via UDP later
        # self.is_triggered = False
        self.get_logger().info("Sequence finished.")


def main(args=None):
    rclpy.init(args=args)
    node = UdpDelayedWrenchPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
