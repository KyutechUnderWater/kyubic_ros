import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StringPublisher(Node):
    def __init__(self):
        super().__init__("string_publisher")
        self.publisher = self.create_publisher(String, "/internship/message", 10)
        self.count = 0
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello ROS 2: {self.count}"
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = StringPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
