import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import pyqtgraph as pg
import pyqtgraph.console
import sys
import threading

from driver_msgs.srv import Command as CommandService


class DVLCommandClient(Node):
    """
    DVLコマンドサービスを呼び出すROS2クライアントノード
    """

    def __init__(self):
        super().__init__("dvl_command_client")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.cli = self.create_client(CommandService, "dvl_command", qos_profile=qos_profile)
        self.get_logger().info("サービス dvl_command を待機中...")

        # サービスが利用可能になるまで待機
        while not self.cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("ROS2がシャットダウンされました。")
                sys.exit(0)
            self.get_logger().info("サービス dvl_command はまだ利用できません。再試行中...")
        self.get_logger().info("サービス dvl_command が利用可能になりました。")

        self._future = None

    def send_command(self, command_str: str) -> str:
        """
        DVLコマンドサービスを呼び出し、レスポンスのoutputを返す
        """
        self.get_logger().info(f'コマンドを送信: "{command_str}"')

        request = CommandService.Request()
        request.command = command_str

        self._future = self.cli.call_async(request)

        # futureが完了するまでブロック (またはrclpy.spin_until_future_completeを使用)
        # コンソール内でこれを呼び出すとUIがブロックされるため、注意が必要
        # この例では、pyqtgraphのコンソールから呼び出されるため、UIスレッドとは別にROS2スレッドが回ることを想定
        rclpy.spin_until_future_complete(self, self._future)

        if self._future.result() is not None:
            response = self._future.result()
            self.get_logger().info(
                f'Service response: success={True if response else False}, output="{
                    response.output
                }"'
            )
            return response.output
        else:
            self.get_logger().error("Service call failed.")
            return "Error: Service call failed."


def rclpy_thread_function(node):
    """
    ROS2のrclpy.spin()を実行するためのスレッド関数
    """
    rclpy.spin(node)
    rclpy.shutdown()


def main():
    rclpy.init(args=sys.argv)

    dvl_client = DVLCommandClient()

    ros_thread = threading.Thread(target=rclpy_thread_function, args=(dvl_client,))
    ros_thread.daemon = True  # メインスレッド終了時に一緒に終了
    ros_thread.start()

    app = pg.mkQApp("DVL Command Console")

    console = pyqtgraph.console.ConsoleWidget(
        namespace={
            "pg": pg,
            "send": dvl_client.send_command,
        },
        text="""
        ====================================================
        DVL Command Console
        ====================================================
        
        Send setting command to DVL via ROS2 Service 'dvl_command'
        
        Example:
        >>> print(send("?"))
        
        ====================================================
        """,
    )
    console.resize(1280, 720)
    console.show()

    sys.exit(pg.exec())


if __name__ == "__main__":
    main()
