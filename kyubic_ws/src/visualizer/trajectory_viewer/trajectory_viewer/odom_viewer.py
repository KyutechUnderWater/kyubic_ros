import rclpy
from rclpy.node import Node

from localization_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading


class OdomPlotter(Node):
    """
    /localization/odomトピックを購読し，ステータスが正常な座標のみをリアルタイムでプロットするノード
    """

    def __init__(self):
        """
        ノードの初期化，サブスクライバとプロットの設定
        """
        super().__init__("odom_plotter_node")

        # 軌跡データを保存するリスト
        self.x_data = []
        self.y_data = []
        # スレッドセーフなデータアクセスのためのロック
        self.data_lock = threading.Lock()

        # エラー状態を示す定数を定義 ( Odometry.msg の定義に基づき ERROR = 2 )
        self.STATUS_ERROR = 2

        # odomトピックのサブスクライバを作成
        self.subscription = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )
        self.get_logger().info(f"'{self.subscription.topic_name}' の購読を開始")

        # Matplotlibのグラフと軸を準備
        self.fig, self.ax = plt.subplots()
        (self.line,) = self.ax.plot([], [], "b-", label="Trajectory")
        (self.point,) = self.ax.plot([], [], "ro", label="Current Position")

        self.setup_plot()

        # アニメーションを作成
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False
        )

    def setup_plot(self):
        """グラフの初期設定"""
        self.ax.set_xlabel("Y Position (m)")
        self.ax.set_ylabel("X Position (m)")
        self.ax.set_title("Odometry Path Plotter")
        self.ax.grid(True)
        self.ax.legend()
        self.ax.set_aspect("equal", adjustable="datalim")

    def odom_callback(self, msg: Odometry):
        """
        トピックからメッセージを受信するたびに呼び出されるコールバック関数
        """
        # ステータスをチェックし，いずれかが 2 (ERROR) の場合はデータを追加しない
        if (
            msg.status.depth == self.STATUS_ERROR
            or msg.status.imu == self.STATUS_ERROR
            or msg.status.dvl == self.STATUS_ERROR
        ):
            self.get_logger().warn(
                "エラー状態のデータを受信したため、プロットをスキップします。"
            )
            return

        # ロックを取得してリストにデータを追加
        with self.data_lock:
            self.x_data.append(msg.pose.position.x)
            self.y_data.append(msg.pose.position.y)

    def update_plot(self, frame):
        """
        描画を更新する関数
        """
        with self.data_lock:
            if not self.x_data:
                return self.line, self.point

            self.line.set_data(self.y_data, self.x_data)
            self.point.set_data([self.y_data[-1]], [self.x_data[-1]])

            self.ax.relim()
            self.ax.autoscale_view()

        return self.line, self.point


def main(args=None):
    rclpy.init(args=args)
    odom_plotter = OdomPlotter()

    spin_thread = threading.Thread(target=rclpy.spin, args=(odom_plotter,), daemon=True)
    spin_thread.start()

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        odom_plotter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
