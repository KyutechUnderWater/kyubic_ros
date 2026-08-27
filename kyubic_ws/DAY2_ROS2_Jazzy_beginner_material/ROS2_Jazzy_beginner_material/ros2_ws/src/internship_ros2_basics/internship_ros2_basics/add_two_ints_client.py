import sys

import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.node import Node


class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.client = self.create_client(AddTwoInts, "/internship/add_two_ints")

    def send_request(self, a, b):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available; waiting...")
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        return self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        print("usage: ros2 run internship_ros2_basics add_two_ints_client A B")
        rclpy.shutdown()
        return

    node = AddTwoIntsClient()
    future = node.send_request(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin_until_future_complete(node, future)
    if future.exception() is not None:
        node.get_logger().error(f"service call failed: {future.exception()}")
    else:
        node.get_logger().info(f"result: {future.result().sum}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
