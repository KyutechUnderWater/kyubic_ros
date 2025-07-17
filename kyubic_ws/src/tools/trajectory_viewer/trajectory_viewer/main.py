#!/home/ros/kyubic_ros/.venv/bin/python
import sys
import os
import threading
import numpy as np
from stl import mesh

# ROS2関連のインポート
import rclpy
from rclpy.node import Node

from localization_msgs.msg import Odometry
from test_pid_msgs.msg import Targets

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
    new_position_signal = pyqtSignal(float, float, float, float, float, float)

    def __init__(self):
        # 親クラスのコンストラクタをそれぞれ呼び出す
        Node.__init__(self, "robot_io_node")
        QObject.__init__(self)

        # --- Subscriber設定 ---
        self.subscription = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

        # --- Publisher設定 ---
        self.targets_pub = self.create_publisher(Targets, "targets", 10)

        # --- 配信データとタイマー ---
        self.target_x_value = 0.0
        self.target_y_value = 0.0
        self.target_z_value = 0.0
        self.target_roll_value = 0.0
        self.target_yaw_value = 0.0
        # 1Hz (1.0秒ごと) で publish_targets を呼び出すタイマー
        self.publisher_timer = self.create_timer(1.0, self.publish_targets)

    def odom_callback(self, msg):
        """'odom' トピックのコールバック"""
        pos = msg.pose.position
        orient = msg.pose.orientation
        self.new_position_signal.emit(
            pos.x, -pos.y, -pos.z_depth, orient.x, orient.y, orient.z
        )

    def publish_targets(self):
        """タイマーによって1Hzで呼び出される関数"""
        msg = Targets()
        msg.x = self.target_x_value
        msg.y = self.target_y_value
        msg.z = self.target_z_value
        msg.roll = self.target_roll_value
        msg.yaw = self.target_yaw_value

        self.targets_pub.publish(msg)

    @pyqtSlot(float, float, float, float, float)
    def update_targets(self, x, y, z, roll, yaw):
        """GUIから呼び出されるスロット。配信する値を更新する。"""
        self.target_x_value = x
        self.target_y_value = y
        self.target_z_value = z
        self.target_roll_value = roll
        self.target_yaw_value = yaw
        self.get_logger().info(
            f"Target values updated: x={x:.2f}, y={y:.2f}, z={z:.2f}, roll={
                roll:.2f}, yaw={yaw:.2f}. Publishing will start/continue."
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

        self.x_axis_item = gl.GLLinePlotItem(
            pos=[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            color=(1.0, 0.0, 0.0, 1.0),
            width=5,
            antialias=True,
        )
        self.y_axis_item = gl.GLLinePlotItem(
            pos=[[0.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
            color=(0.0, 1.0, 0.0, 1.0),
            width=5,
            antialias=True,
        )
        self.z_axis_item = gl.GLLinePlotItem(
            pos=[[0.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
            color=(0.0, 0.0, 1.0, 1.0),
            width=5,
            antialias=True,
        )
        self.addItem(self.x_axis_item)
        self.addItem(self.y_axis_item)
        self.addItem(self.z_axis_item)

        self.trajectory_points = np.empty((0, 3))
        self.trajectory_points = np.vstack(
            [self.trajectory_points, np.array([0.0, 0.0, 0.0])]
        )
        self.attitude = np.zeros(3)

        self.plot_item = gl.GLLinePlotItem(
            pos=self.trajectory_points,
            color=(1.0, 0.5, 0.0, 1.0),
            width=5,
            antialias=True,
        )
        self.addItem(self.plot_item)

        self.mesh = None
        self.showSTL(f"{os.path.dirname(__file__)
                        }/../assets/KYUBIC_transformed.stl")

    def loadSTL(self, filename):
        m = mesh.Mesh.from_file(filename)

        # Scale
        scale = 1.0
        m.x = m.x * scale
        m.y = m.y * scale
        m.z = m.z * scale

        # Position
        # offset_z = m.max_ - m.min_
        # m.z = m.z - offset_z

        print(f"point.shape: {m.points.shape}")

        # Rotation
        rot = 0
        rot_z_matrix = np.array(
            [
                [np.cos(np.deg2rad(rot)), -np.sin(np.deg2rad(rot)), 0],
                [np.sin(np.deg2rad(rot)), np.cos(np.deg2rad(rot)), 0],
                [0, 0, 1],
            ]
        )
        R = np.zeros((4, 4))
        R[:3, :3] = rot_z_matrix
        m.transform(R)

        points = m.points.reshape(-1, 3)
        faces = np.arange(points.shape[0]).reshape(-1, 3)
        return points, faces

    def showSTL(self, filename):
        points, faces = self.loadSTL(filename)
        meshdata = gl.MeshData(vertexes=points, faces=faces)
        self.mesh = gl.GLMeshItem(
            meshdata=meshdata,
            smooth=True,
            drawFaces=True,
            drawEdges=True,
            color=(0.8, 0.8, 0.8, 1),
            edgeColor=(0.5, 0.5, 0.5, 1),
        )
        self.addItem(self.mesh)

    @pyqtSlot(float, float, float, float, float, float)
    def add_point(self, x, y, z, roll, pitch, yaw):
        """軌跡に新しい点を追加してプロットを更新するスロット"""
        new_point = np.array([[x, y, z]])
        new_attitude = np.array((roll, pitch, yaw))

        # 前回と今回の値の差分ベクトルのL2ノルムが 0.2[m] より大きいとき，5度よりの大きい回転があったとき更新
        if np.linalg.norm(self.trajectory_points[-1] - new_point, ord=2) > 0.01 or any(
            np.abs(new_attitude - self.attitude) > 1
        ):
            self.trajectory_points = np.vstack(
                [self.trajectory_points, new_point])
            self.plot_item.setData(pos=self.trajectory_points)

            self.attitude = new_attitude

            # Transform3Dオブジェクトを作成
            tr = pg.Transform3D()

            # 移動を適用
            tr.translate(*new_point.reshape(-1))

            # 回転を適用 (Z-Y-X順で適用するため、メソッドはX-Y-Zの逆順で呼び出す)
            # 1. ロール (X軸周り)
            tr.rotate(roll, 1, 0, 0)
            # 2. ピッチ (Y軸周り)
            tr.rotate(pitch, 0, 1, 0)
            # 3. ヨー (Z軸周り)
            tr.rotate(yaw, 0, 0, 1)

            # メッシュにTransform3Dを適用
            self.mesh.setTransform(tr)


class MainWindow(QMainWindow):
    """
    メインウィンドウ。プロッタと制御パネルをレイアウトする。
    """

    # 決定ボタンが押されたときに発行されるシグナル (x, y, z, roll, yaw)
    targets_submitted = pyqtSignal(float, float, float, float, float)

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

        # x軸位置入力
        control_layout.addWidget(QLabel("<b>Target Pos X [m]</b>"))
        self.x_input = QDoubleSpinBox()
        self.x_input.setRange(-15.0, 15.0)
        self.x_input.setSingleStep(0.1)
        self.x_input.setValue(0.0)
        control_layout.addWidget(self.x_input)

        # y軸位置入力
        control_layout.addWidget(QLabel("<b>Target Pos Y [m]</b>"))
        self.y_input = QDoubleSpinBox()
        self.y_input.setRange(-15.0, 15.0)
        self.y_input.setSingleStep(0.1)
        self.y_input.setValue(0.0)
        control_layout.addWidget(self.y_input)

        # z軸位置入力
        control_layout.addWidget(QLabel("<b>Target Pos Z [m]</b>"))
        self.z_input = QDoubleSpinBox()
        self.z_input.setRange(0.0, 15.0)
        self.z_input.setSingleStep(0.1)
        self.z_input.setValue(0.0)
        control_layout.addWidget(self.z_input)

        # roll軸角度入力
        control_layout.addWidget(QLabel("<b>Target Roll[deg]</b>"))
        self.roll_input = QDoubleSpinBox()
        self.roll_input.setRange(-30.0, 30.0)
        self.roll_input.setSingleStep(1.0)
        self.roll_input.setValue(0.0)
        control_layout.addWidget(self.roll_input)

        # yaw軸角度入力
        control_layout.addWidget(QLabel("<b>Target Heading[deg]</b>"))
        self.yaw_input = QDoubleSpinBox()
        self.yaw_input.setRange(-180.0, 180.0)
        self.yaw_input.setSingleStep(1.0)
        self.yaw_input.setValue(0.0)
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
        x = self.x_input.value()
        y = self.y_input.value()
        z = self.z_input.value()
        roll = self.roll_input.value()
        yaw = self.yaw_input.value()
        self.targets_submitted.emit(x, y, z, roll, yaw)


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
