import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from turtlesim.action import RotateAbsolute


class TurtleRotateActionClient(Node):
    def __init__(self):
        super().__init__("turtle_rotate_action_client")
        self.client = ActionClient(
            self,
            RotateAbsolute,
            "/turtle1/rotate_absolute",
        )

    def send_goal(self, degrees):
        goal = RotateAbsolute.Goal()
        goal.theta = math.radians(degrees)

        self.get_logger().info("waiting for /turtle1/rotate_absolute...")
        self.client.wait_for_server()
        future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("goal rejected")
            rclpy.shutdown()
            return

        self.get_logger().info("goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        remaining = math.degrees(feedback_msg.feedback.remaining)
        self.get_logger().info(f"remaining: {remaining:.1f} deg")

    def result_callback(self, future):
        result = future.result().result
        delta = math.degrees(result.delta)
        self.get_logger().info(f"finished; rotated delta={delta:.1f} deg")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 2:
        print("usage: ros2 run internship_ros2_basics turtle_rotate_action_client DEGREES")
        rclpy.shutdown()
        return

    node = TurtleRotateActionClient()
    node.send_goal(float(sys.argv[1]))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
