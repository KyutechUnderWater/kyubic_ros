import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn


class SpawnTurtleClientExercise(Node):
    def __init__(self):
        super().__init__("spawn_turtle_client_exercise")

        # TODO 1: Spawn型、/spawnのclientを作る
        self.client = None

    def send_request(self):
        # TODO 2: serviceが現れるまでwait_for_serviceする
        raise NotImplementedError("TODO 2")

        request = Spawn.Request()

        # TODO 3: x=2.0, y=2.0, theta=0.0, name='student_turtle'を設定する

        # TODO 4: call_asyncでrequestを送ってfutureを返す
        raise NotImplementedError("TODO 4")


def main(args=None):
    rclpy.init(args=args)
    node = SpawnTurtleClientExercise()
    future = node.send_request()
    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info(f"spawned: {future.result().name}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
