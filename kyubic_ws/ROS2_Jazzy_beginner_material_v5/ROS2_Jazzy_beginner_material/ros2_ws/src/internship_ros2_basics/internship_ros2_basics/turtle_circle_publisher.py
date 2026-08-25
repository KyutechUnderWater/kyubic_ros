import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TurtleCirclePublisher(Node):
    def __init__(self):
        super().__init__("turtle_circle_publisher")
        self.declare_parameter("linear_speed", 1.5)
        self.declare_parameter("angular_speed", 1.0)
        self.publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = float(self.get_parameter("linear_speed").value)
        msg.angular.z = float(self.get_parameter("angular_speed").value)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCirclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.publisher.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
