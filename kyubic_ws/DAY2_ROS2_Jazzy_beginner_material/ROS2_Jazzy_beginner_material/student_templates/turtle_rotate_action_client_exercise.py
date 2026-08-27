import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from turtlesim.action import RotateAbsolute


class TurtleRotateActionClientExercise(Node):
    def __init__(self):
        super().__init__("turtle_rotate_action_client_exercise")

        # TODO 1: RotateAbsolute型、/turtle1/rotate_absoluteのActionClientを作る
        self.client = None

    def send_goal(self, degrees):
        goal = RotateAbsolute.Goal()

        # TODO 2: degreesをradへ変換してgoal.thetaへ入れる
        goal.theta = 0.0

        # TODO 3: action serverを待つ
        raise NotImplementedError("TODO 3")

        # TODO 4: feedback_callback付きでgoalを非同期送信する
        future = None

        # TODO 5: goal_response_callbackをdone callbackへ登録する
        return future

    def goal_response_callback(self, future):
        goal_handle = future.result()

        # TODO 6: rejectedの場合を処理する

        # TODO 7: get_result_asyncしてresult_callbackを登録する

    def feedback_callback(self, feedback_msg):
        remaining_deg = math.degrees(feedback_msg.feedback.remaining)
        self.get_logger().info(f"remaining={remaining_deg:.1f} deg")

    def result_callback(self, future):
        delta_deg = math.degrees(future.result().result.delta)
        self.get_logger().info(f"finished delta={delta_deg:.1f} deg")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TurtleRotateActionClientExercise()
    node.send_goal(90.0)
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
