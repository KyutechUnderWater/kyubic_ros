import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StringPublisherExercise(Node):
    def __init__(self):
        # TODO 1: ノード名を指定してNodeを初期化する
        super().__init__("TODO_node_name")

        # TODO 2: String型、/internship/message、depth 10のpublisherを作る
        self.publisher = None
        self.count = 0

        # TODO 3: 0.5秒ごとにtimer_callbackを呼ぶtimerを作る
        self.timer = None

    def timer_callback(self):
        msg = String()
        # TODO 4: countを含む文字列をmsg.dataへ入れる
        msg.data = "TODO"

        # TODO 5: メッセージをpublishし、countを1増やす
        raise NotImplementedError("TODO 5")


def main(args=None):
    rclpy.init(args=args)
    node = StringPublisherExercise()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
