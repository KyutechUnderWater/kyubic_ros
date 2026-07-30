"""Adapter node: BuoyRelativePosition (perception/buoy_detector) -> BuoyDetection.

bluerov_control_bt_nodes's CheckBuoyDetected subscribes to BuoyDetection
(bluerov_control_msgs), but the image-processing team's buoy_detector_node publishes
BuoyRelativePosition (buoy_interfaces) instead. This node republishes one as the other so
CheckBuoyDetected has a real publisher to talk to. It performs no coordinate transform (that
is already done by buoy_detector_node/buoy_geometry.py) and holds no state across messages:
each BuoyRelativePosition message is converted and republished independently.

Subscribe-and-republish only. This node cannot actuate the vehicle.
"""

from bluerov_control_msgs.msg import BuoyDetection
from buoy_interfaces.msg import BuoyRelativePosition
import rclpy
from rclpy.node import Node


class BuoyDetectorDriver(Node):
    """Converts BuoyRelativePosition into BuoyDetection, one message at a time."""

    def __init__(self) -> None:
        """Declare parameters and wire up the subscription/publisher."""
        super().__init__("buoy_detector_driver")

        self.declare_parameter("input_topic", "/buoy/relative_position")
        self.declare_parameter("output_topic", "/perception/buoy_detection")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self._publisher = self.create_publisher(BuoyDetection, output_topic, 10)
        self._subscription = self.create_subscription(
            BuoyRelativePosition, input_topic, self._relative_position_callback, 10
        )

        self.get_logger().info(
            f"buoy_detector_driver started: input={input_topic} output={output_topic}"
        )

    def _relative_position_callback(self, msg: BuoyRelativePosition) -> None:
        # Statusマッピング(NORMAL/ERROR相当。WARNINGは使わない):
        #   detected=false                        -> ERROR (何も見えていない)
        #   detected=true, position_valid=false    -> ERROR (検出したが座標は信用できない)
        #   detected=true, position_valid=true     -> NORMAL
        # 直前値は保持しない: 次のフレームでまた検出を試みればよいため、
        # ERROR判定の周期は正直にdetected=falseとして報告する。
        is_normal = msg.detected and msg.position_valid

        out = BuoyDetection()
        out.header = msg.header
        out.detected = is_normal
        out.confidence = msg.confidence
        if is_normal:
            out.relative_buoy_x_m = msg.relative_buoy_x_m
            out.relative_buoy_y_m = msg.relative_buoy_y_m
            out.relative_buoy_z_m = msg.relative_buoy_z_m
        else:
            nan = float("nan")
            out.relative_buoy_x_m = nan
            out.relative_buoy_y_m = nan
            out.relative_buoy_z_m = nan

        self._publisher.publish(out)


def main(args: list[str] | None = None) -> None:
    """Run the buoy_detector_driver node until shutdown."""
    rclpy.init(args=args)
    node = BuoyDetectorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
