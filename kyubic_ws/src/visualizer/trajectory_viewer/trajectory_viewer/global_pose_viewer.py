import rclpy
from rclpy.node import Node

from localization_msgs.msg import GlobalPose
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading


class GlobalPosePlotter(Node):
    """
    /localization/global_poseトピックを購読し，緯度経度をリアルタイムでプロットするノード
    """

    def __init__(self):
        """
        ノードの初期化、サブスクライバとプロットの設定
        """
        super().__init__("global_pose_viewer_node")

        anchor_lat: list[float] = self.declare_parameter("anchor_lat", [0.0]).value
        anchor_lon: list[float] = self.declare_parameter("anchor_lon", [0.0]).value

        # 軌跡データを保存するリスト
        self.lon_data = []  # 経度 (Longitude) - X軸
        self.lat_data = []  # 緯度 (Latitude)  - Y軸
        # スレッドセーフなデータアクセスのためのロック
        self.data_lock = threading.Lock()

        # エラー状態を示す定数 ( GlobalPose.msg の定義に基づき ERROR = 2 )
        self.STATUS_ERROR = 2

        # /localization/global_pose トピックのサブスクライバを作成
        self.subscription = self.create_subscription(
            GlobalPose, "global_pose", self.pose_callback, 10
        )
        self.get_logger().info(f"'{self.subscription.topic_name}' の購読を開始")

        # Matplotlibのグラフと軸を準備
        self.fig, self.ax = plt.subplots()
        (self.line,) = self.ax.plot([], [], "b-", label="Trajectory")
        (self.point,) = self.ax.plot([], [], "ro", label="Local area")
        # self.ax.plot(anchor_lon, anchor_lat, "ro", label="anchor")

        self.setup_plot()

        # アニメーションを作成
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False, save_count=1000
        )

    def setup_plot(self):
        """グラフの初期設定"""
        self.ax.set_xlabel("Longitude")
        self.ax.set_ylabel("Latitude")
        self.ax.set_title("Geodetic Path Plotter (Lat/Lon)")
        self.ax.grid(True)
        self.ax.legend()
        # 緯度経度プロットではアスペクト比を自動調整させることが多い
        # 見やすいように 'auto' に設定。'equal' に戻すことも可能。
        self.ax.set_aspect("auto", adjustable="datalim")
        # 軸のフォーマットを小数点以下が多いことを想定して調整
        self.ax.ticklabel_format(style="plain", useOffset=False)

    def pose_callback(self, msg: GlobalPose):
        """
        トピックからメッセージを受信するたびに呼び出されるコールバック関数
        """
        # ステータスをチェックし、いずれかが 2 (ERROR) の場合はデータを追加しない
        if (
            msg.status.depth.id == self.STATUS_ERROR
            or msg.status.imu.id == self.STATUS_ERROR
            or msg.status.dvl.id == self.STATUS_ERROR
        ):
            self.get_logger().warn("エラー状態のデータを受信したため、プロットをスキップします。")
            return

        # ロックを取得してリストにデータを追加
        with self.data_lock:
            # 経度をX軸、緯度をY軸として追加
            self.lon_data.append(msg.current_pose.longitude)
            self.lat_data.append(msg.current_pose.latitude)

    def update_plot(self, frame):
        """
        アニメーションのフレームを更新する関数
        """
        with self.data_lock:
            if not self.lon_data:
                return self.line, self.point

            self.line.set_data(self.lon_data, self.lat_data)
            self.point.set_data([self.lon_data[-1]], [self.lat_data[-1]])

            self.ax.relim()
            self.ax.autoscale_view()

            # 軸ラベルが回転しないように調整
            plt.setp(self.ax.get_xticklabels(), rotation=10, ha="right")
            plt.setp(self.ax.get_yticklabels(), rotation=10, ha="right")

        return self.line, self.point


def main(args=None):
    rclpy.init(args=args)
    global_pose_plotter = GlobalPosePlotter()

    spin_thread = threading.Thread(target=rclpy.spin, args=(global_pose_plotter,), daemon=True)
    spin_thread.start()

    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        global_pose_plotter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
