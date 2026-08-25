import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class TurtlePoseSubscriber(Node):
    def __init__(self):
        super().__init__("turtle_pose_subscriber")
        self.subscription = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.pose_callback,
            10,
        )

    def pose_callback(self, msg):
        self.get_logger().info(
            f"x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}, "
            f"v={msg.linear_velocity:.2f}, omega={msg.angular_velocity:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
