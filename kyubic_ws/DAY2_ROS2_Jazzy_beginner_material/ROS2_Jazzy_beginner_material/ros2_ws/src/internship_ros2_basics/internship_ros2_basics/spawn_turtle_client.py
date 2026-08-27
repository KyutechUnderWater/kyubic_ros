import sys

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn


class SpawnTurtleClient(Node):
    def __init__(self):
        super().__init__("spawn_turtle_client")
        self.client = self.create_client(Spawn, "/spawn")

    def send_request(self, x, y, theta, name):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/spawn is not available; is turtlesim running?")
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        return self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 5:
        print("usage: ros2 run internship_ros2_basics spawn_turtle_client X Y THETA NAME")
        rclpy.shutdown()
        return

    node = SpawnTurtleClient()
    future = node.send_request(
        float(sys.argv[1]),
        float(sys.argv[2]),
        float(sys.argv[3]),
        sys.argv[4],
    )
    rclpy.spin_until_future_complete(node, future)
    if future.exception() is not None:
        node.get_logger().error(f"spawn failed: {future.exception()}")
    else:
        node.get_logger().info(f"spawned turtle: {future.result().name}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
