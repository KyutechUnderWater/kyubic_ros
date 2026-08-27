import sys

import rclpy
from action_msgs.msg import GoalStatus
from internship_interfaces.action import Fibonacci
from rclpy.action import ActionClient
from rclpy.node import Node


class FibonacciActionClient(Node):
    def __init__(self, cancel_after_sec=0.0):
        super().__init__("fibonacci_action_client")
        self.client = ActionClient(self, Fibonacci, "/internship/fibonacci")
        self.cancel_after_sec = cancel_after_sec
        self.goal_handle = None
        self.cancel_timer = None

    def send_goal(self, order):
        goal = Fibonacci.Goal()
        goal.order = order

        self.get_logger().info("waiting for action server...")
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
        self.goal_handle = goal_handle
        if self.cancel_after_sec > 0.0:
            self.cancel_timer = self.create_timer(
                self.cancel_after_sec,
                self.cancel_goal,
            )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def cancel_goal(self):
        if self.cancel_timer is not None:
            self.cancel_timer.cancel()
        if self.goal_handle is None:
            return
        self.get_logger().warn("sending cancel request")
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        response = future.result()
        self.get_logger().info(f"cancel response: goals_canceling={len(response.goals_canceling)}")

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"feedback: {list(feedback_msg.feedback.partial_sequence)}")

    def result_callback(self, future):
        wrapped_result = future.result()
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }.get(wrapped_result.status, str(wrapped_result.status))
        self.get_logger().info(f"result ({status_name}): {list(wrapped_result.result.sequence)}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) not in {2, 3}:
        print(
            "usage: ros2 run internship_ros2_basics "
            "fibonacci_action_client ORDER [CANCEL_AFTER_SEC]"
        )
        rclpy.shutdown()
        return

    cancel_after = float(sys.argv[2]) if len(sys.argv) == 3 else 0.0
    node = FibonacciActionClient(cancel_after)
    node.send_goal(int(sys.argv[1]))
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
