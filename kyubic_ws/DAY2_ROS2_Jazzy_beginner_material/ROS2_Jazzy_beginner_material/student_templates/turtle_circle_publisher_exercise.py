import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TurtleCirclePublisherExercise(Node):
    def __init__(self):
        super().__init__("turtle_circle_publisher_exercise")

        # TODO 1: /turtle1/cmd_velへTwistを送るpublisherを作る
        self.publisher = None

        # TODO 2: 0.1秒周期のtimerを作る
        self.timer = None

    def timer_callback(self):
        msg = Twist()

        # TODO 3: 前進速度を1.5、角速度を1.0にする
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        # TODO 4: publishする
        raise NotImplementedError("TODO 4")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCirclePublisherExercise()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
