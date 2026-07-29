"""mavlink_driverのcontrol_heartbeat_timeout_s向けに、heartbeat(std_msgs/Bool)を
一定周期でpublishし続けるだけの独立ノード。

bluerov_control_node(FSM+PID)を廃止してwrench_planner構成(BT)に置き換える際、
robot_forceの送信元がbluerov_control_nodeでは無くなるため、heartbeatの送信元も
同様に切り出す必要がある。ロジックはbluerov_control_node._publish_heartbeatと同じ
(常時true。mission状態やrobot_forceの新鮮さとは連動しない、mavlink_driver側の
command_timeout_sによるrobot_force鮮度チェックとは独立した安全レイヤー)。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

HEARTBEAT_PERIOD_SEC = 0.2  # mavlink_driverのcontrol_heartbeat_timeout_s(既定1.0s)より十分短い


class HeartbeatPublisher(Node):
    def __init__(self) -> None:
        super().__init__("bluerov_heartbeat_publisher")
        self._pub = self.create_publisher(Bool, "heartbeat", 10)
        self.create_timer(HEARTBEAT_PERIOD_SEC, self._publish_heartbeat)
        self.get_logger().info("bluerov_heartbeat_publisher を起動しました")

    def _publish_heartbeat(self) -> None:
        msg = Bool()
        msg.data = True
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeartbeatPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
