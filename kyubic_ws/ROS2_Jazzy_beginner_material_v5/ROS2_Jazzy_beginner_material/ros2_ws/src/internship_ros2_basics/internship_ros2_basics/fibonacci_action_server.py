import time

import rclpy
from internship_interfaces.action import Fibonacci
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__("fibonacci_action_server")
        callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(
            self,
            Fibonacci,
            "/internship/fibonacci",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group,
        )

    def goal_callback(self, goal_request):
        if goal_request.order < 1 or goal_request.order > 30:
            self.get_logger().warn("rejecting order; expected 1..30")
            return GoalResponse.REJECT
        self.get_logger().info(f"accepting order={goal_request.order}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self.get_logger().info("cancel request accepted")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        feedback = Fibonacci.Feedback()
        feedback.partial_sequence = [0]
        if goal_handle.request.order >= 2:
            feedback.partial_sequence.append(1)
        goal_handle.publish_feedback(feedback)

        while len(feedback.partial_sequence) < goal_handle.request.order:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Fibonacci.Result()
                result.sequence = feedback.partial_sequence
                self.get_logger().warn("goal canceled")
                return result

            feedback.partial_sequence.append(
                feedback.partial_sequence[-1] + feedback.partial_sequence[-2]
            )
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"feedback: {feedback.partial_sequence}")
            time.sleep(0.5)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback.partial_sequence
        self.get_logger().info("goal succeeded")
        return result

    def destroy_node(self):
        self.action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
