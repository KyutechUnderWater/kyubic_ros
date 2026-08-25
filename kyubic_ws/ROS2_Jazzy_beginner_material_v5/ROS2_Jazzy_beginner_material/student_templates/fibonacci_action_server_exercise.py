import time

import rclpy
from internship_interfaces.action import Fibonacci
from rclpy.action import ActionServer
from rclpy.node import Node


class FibonacciActionServerExercise(Node):
    def __init__(self):
        super().__init__("fibonacci_action_server_exercise")

        # TODO 1: Fibonacci型、/internship/fibonacci、execute_callbackでActionServerを作る
        self.action_server = None

    def execute_callback(self, goal_handle):
        feedback = Fibonacci.Feedback()
        feedback.partial_sequence = [0]
        if goal_handle.request.order >= 2:
            feedback.partial_sequence.append(1)

        while len(feedback.partial_sequence) < goal_handle.request.order:
            # TODO 2: 次のFibonacci数をpartial_sequenceへ追加する

            # TODO 3: feedbackをpublishする

            time.sleep(0.5)

        # TODO 4: goalを成功状態にする

        result = Fibonacci.Result()

        # TODO 5: 最終sequenceをresultへ入れて返す
        raise NotImplementedError("TODO 5")


def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServerExercise()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
