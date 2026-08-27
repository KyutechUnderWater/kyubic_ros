import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.node import Node


class AddTwoIntsServerExercise(Node):
    def __init__(self):
        super().__init__("add_two_ints_server_exercise")

        # TODO 1: AddTwoInts型、/internship/add_two_ints、add_callbackを指定する
        self.service = None

    def add_callback(self, request, response):
        # TODO 2: request.aとrequest.bを足してresponse.sumへ入れる
        response.sum = 0

        # TODO 3: responseを返す
        raise NotImplementedError("TODO 3")


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServerExercise()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
