#!/home/ros/kyubic_ros/.venv/bin/python
import sys
import threading
import numpy as np

# ROS2関連のインポート
import rclpy
from rclpy.node import Node

from localization_msgs.msg import Odometry
from std_msgs.msg import Float32  # Float32メッセージ型を追加

# PyQtとPyQtGraph関連のインポート
from PyQt6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QDoubleSpinBox,
)
from PyQt6.QtCore import pyqtSignal, QObject, pyqtSlot
import pyqtgraph.opengl as gl
import pyqtgraph as pg


class RosCommunicator(Node, QObject):
    """
    ROS2の通信（購読と配信）を管理するクラス。
    """

    # 軌跡プロット用の位置データシグナル
    new_position_signal = pyqtSignal(float, float, float)

    def __init__(self):
        # 親クラスのコンストラクタをそれぞれ呼び出す
        Node.__init__(self, "robot_io_node")
        QObject.__init__(self)

        # --- Subscriber設定 ---
        self.subscription = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

        # --- Publisher設定 ---
        self.target_z_pub = self.create_publisher(Float32, "target_z", 10)
        self.target_yaw_pub = self.create_publisher(Float32, "target_yaw", 10)

        # --- 配信データとタイマー ---
        self.target_z_value = None
        self.target_yaw_value = None
        # 1Hz (1.0秒ごと) で publish_targets を呼び出すタイマー
        self.publisher_timer = self.create_timer(1.0, self.publish_targets)

    def odom_callback(self, msg):
        """'odom' トピックのコールバック"""
        pos = msg.pose.position
        self.new_position_signal.emit(pos.x, pos.y, -pos.z_depth)

    def publish_targets(self):
        """タイマーによって1Hzで呼び出される関数"""
        # z軸目標値が設定されていれば配信
        if self.target_z_value is not None:
            msg = Float32()
            msg.data = self.target_z_value
            self.target_z_pub.publish(msg)

        # yaw軸目標値が設定されていれば配信
        if self.target_yaw_value is not None:
            msg = Float32()
            msg.data = self.target_yaw_value
            self.target_yaw_pub.publish(msg)

    @pyqtSlot(float, float)
    def update_targets(self, z, yaw):
        """GUIから呼び出されるスロット。配信する値を更新する。"""
        self.target_z_value = z
        self.target_yaw_value = yaw
        self.get_logger().info(
            f"Target values updated: z={z:.2f}, yaw={
                yaw:.2f}. Publishing will start/continue."
        )


class TrajectoryPlotter(gl.GLViewWidget):
    """3Dプロット用ウィジェット（変更なし）"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setCameraPosition(distance=5, elevation=0, azimuth=45)

        # Create Grid
        grid = gl.GLGridItem()
        grid.setSize(30, 30)
        grid.setSpacing(1, 1)
        self.addItem(grid)

        self.trajectory_points = np.empty((0, 3))
        self.trajectory_points = np.vstack(
            [self.trajectory_points, np.array([0.0, 0.0, 0.0])]
        )

        self.plot_item = gl.GLLinePlotItem(
            pos=self.trajectory_points,
            color=(1.0, 0.5, 0.0, 1.0),
            width=5,
            antialias=True,
        )
        self.addItem(self.plot_item)

    @pyqtSlot(float, float, float)
    def add_point(self, x, y, z):
        """軌跡に新しい点を追加してプロットを更新するスロット"""
        new_point = np.array([[x, y, z]])

        # 前回と今回の値の差分ベクトルのL2ノルムが 0.2[m] より大きいときに更新
        if np.linalg.norm(self.trajectory_points[-1] - new_point, ord=2) > 0.1:
            self.trajectory_points = np.vstack(
                [self.trajectory_points, new_point])
            self.plot_item.setData(pos=self.trajectory_points)


class MainWindow(QMainWindow):
    """
    メインウィンドウ。プロッタと制御パネルをレイアウトする。
    """

    # 決定ボタンが押されたときに発行されるシグナル (z, yaw)
    targets_submitted = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("水中ロボット 3D航路プロッタ & コントローラ")

        # --- メインレイアウト ---
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # --- 左側: 3Dプロッタ ---
        self.plotter = TrajectoryPlotter()
        main_layout.addWidget(self.plotter, 3)  # レイアウト比率を3に設定

        # --- 右側: 制御パネル ---
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        main_layout.addWidget(control_panel, 1)  # レイアウト比率を1に設定

        # z軸位置入力
        control_layout.addWidget(QLabel("<b>Target Pos [m]</b>"))
        self.z_input = QDoubleSpinBox()
        self.z_input.setRange(0.0, 15.0)
        self.z_input.setSingleStep(0.1)
        self.z_input.setValue(-1.0)
        control_layout.addWidget(self.z_input)

        # yaw軸角度入力
        control_layout.addWidget(QLabel("<b>Target Heading[deg]</b>"))
        self.yaw_input = QDoubleSpinBox()
        self.yaw_input.setRange(-180.0, 180.0)
        self.yaw_input.setSingleStep(1.0)
        self.yaw_input.setValue(90.0)
        control_layout.addWidget(self.yaw_input)

        control_layout.addSpacing(20)

        # 決定ボタン
        self.submit_button = QPushButton("Set Values")
        self.submit_button.setStyleSheet("padding: 10px; font-weight: bold;")
        self.submit_button.clicked.connect(self.on_submit)
        control_layout.addWidget(self.submit_button)

        # ウィジェットが上部に寄るようにスペーサーを追加
        control_layout.addStretch()

    def on_submit(self):
        """決定ボタンがクリックされたときの処理"""
        z = self.z_input.value()
        yaw = self.yaw_input.value()
        self.targets_submitted.emit(z, yaw)


def main(args=None):
    # 1. ROS2の初期化
    rclpy.init(args=args)

    # 2. PyQtアプリケーションの作成
    pg.mkQApp("Trajectory Viewer")

    # 3. メインウィンドウとROS通信クラスのインスタンス化
    main_window = MainWindow()
    ros_communicator = RosCommunicator()

    # 4. シグナルとスロットを接続
    # 4-1. ROS(Odom) -> プロッタ描画
    ros_communicator.new_position_signal.connect(main_window.plotter.add_point)
    # 4-2. GUI(ボタン) -> ROS(目標値更新)
    main_window.targets_submitted.connect(ros_communicator.update_targets)

    # 5. ROSノードを別スレッドで実行
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(ros_communicator)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # 6. メインウィンドウを表示
    main_window.show()
    main_window.resize(1200, 800)

    # 7. PyQtアプリケーションの実行と終了処理
    try:
        status = pg.exec()
    finally:
        print("Shutting down...")
        ros_communicator.destroy_node()
        rclpy.shutdown()
        ros_thread.join()

    sys.exit(status)


if __name__ == "__main__":
    main()
