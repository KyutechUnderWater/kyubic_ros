import sys
import threading
import collections
import numpy as np
import time
from functools import partial

# ROS2関連
import rclpy
from rclpy.node import Node
from localization_msgs.msg import Odometry
from std_msgs.msg import Float32

# PyQt5/PyQtGraph関連
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QGridLayout,
)
from PyQt5.QtCore import QTimer

# PyQtGraphの基本設定
import pyqtgraph as pg

pg.setConfigOption("background", "w")
pg.setConfigOption("foreground", "k")

# プロットするデータの最大保持数
MAX_DATA_POINTS = 50  # subscriber 5Hz -> buffer 10s


class PlotUnitWidget(QWidget):
    """
    一つのグラフと、その設定UIをまとめたウィジェットクラス
    """

    def __init__(self, title, unit_label, parent=None):
        super(PlotUnitWidget, self).__init__(parent)
        layout = QVBoxLayout()
        self.setLayout(layout)

        # --- Y軸範囲設定UI ---
        yrange_layout = QHBoxLayout()
        yrange_layout.addWidget(QLabel(f"{title} Y-axis | min:"))
        self.min_y_input = QLineEdit("-0.4")
        yrange_layout.addWidget(self.min_y_input)

        yrange_layout.addWidget(QLabel("max:"))
        self.max_y_input = QLineEdit("0.4")
        yrange_layout.addWidget(self.max_y_input)

        set_range_button = QPushButton("Set range")
        set_range_button.clicked.connect(self.update_yrange)
        yrange_layout.addWidget(set_range_button)

        # --- グラフウィジェット ---
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setLabel("left", title, units=unit_label)
        self.plot_widget.setLabel("bottom", "Time", units="s")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.2)
        self.plot_widget.addLegend()

        self.value_curve = self.plot_widget.plot(
            pen=pg.mkPen("b", width=2), name="Value"
        )
        self.target_curve = self.plot_widget.plot(
            pen=pg.mkPen("r", width=2, style=pg.QtCore.Qt.DashLine), name="Target"
        )

        layout.addLayout(yrange_layout)
        layout.addWidget(self.plot_widget)

        self.update_yrange()

    def setData(self, time_data, value_data, target_data):
        self.value_curve.setData(time_data, value_data)
        self.target_curve.setData(time_data, target_data)

    def update_yrange(self):
        try:
            min_y = float(self.min_y_input.text())
            max_y = float(self.max_y_input.text())
            if min_y < max_y:
                self.plot_widget.setYRange(min_y, max_y)
        except ValueError:
            pass  # 不正な値は無視


class MultiGraphViewer(QMainWindow):
    """
    複数のグラフを統括するメインウィンドウクラス
    """

    def __init__(self, ros_node, parent=None):
        super(MultiGraphViewer, self).__init__(parent)
        self.ros_node = ros_node
        self.start_time = time.time()

        self.setWindowTitle("Odometry Realtime Viewer")
        self.setGeometry(100, 100, 1200, 800)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        grid_layout = QGridLayout()
        central_widget.setLayout(grid_layout)

        # --- 4つのグラフエリアを作成 ---
        plot_configs = [
            # {"title": "Velocity X", "unit": "m/s"},
            # {"title": "Velocity Y", "unit": "m/s"},
            {"title": "Position Z", "unit": "m"},
            {"title": "Direction Z", "unit": "deg"},
            {"title": "Velocity Z", "unit": "m/s"},
            {"title": "Angular Vel Z", "unit": "deg/s"},
        ]

        self.plot_units = []
        for i, config in enumerate(plot_configs):
            plot_unit = PlotUnitWidget(config["title"], config["unit"])
            self.plot_units.append(plot_unit)
            # グリッドレイアウトに配置 (2x2)
            grid_layout.addWidget(plot_unit, i // 2, i % 2)

        # --- データ保持用バッファ ---
        self.time_data = collections.deque(maxlen=MAX_DATA_POINTS)
        # 0:vel_x, 1:vel_y, 2:vel_z, 3:ang_z
        self.value_data = [collections.deque(maxlen=MAX_DATA_POINTS) for _ in range(4)]
        self.target_data = [collections.deque(maxlen=MAX_DATA_POINTS) for _ in range(4)]

        # --- グラフ更新用タイマー ---
        self.timer = QTimer()
        self.timer.setInterval(50)  # 50msごとに更新 (20Hz)
        self.timer.timeout.connect(self.update_plots)
        self.timer.start()

    def update_plots(self):
        new_data_points = self.ros_node.get_data_points()

        if not new_data_points:
            return

        print(len(new_data_points))
        for data_tuple in new_data_points:
            timestamp, values, targets = data_tuple
            self.time_data.append(timestamp - self.start_time)
            for i in range(4):
                self.value_data[i].append(values[i])
                self.target_data[i].append(targets[i])

        time_list = list(self.time_data)
        for i in range(4):
            self.plot_units[i].setData(
                time_list, list(self.value_data[i]), list(self.target_data[i])
            )


class MultiDimSubscriber(Node):
    """
    Odometryと複数の目標値トピックを購読するROS2ノードクラス
    """

    def __init__(self):
        super().__init__("multi_dim_plotter_node")

        self.data_queue = collections.deque(maxlen=100)
        self.lock = threading.Lock()

        # 4つの目標値を辞書で管理
        self.latest_targets = {"vel_x": 0.0, "vel_y": 0.0, "vel_z": 0.0, "ang_z": 0.0}

        # サブスクライバの定義
        self.odom_subscription = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

        target_topics = {
            "vel_x": "/target_vel_x",
            "vel_y": "/target_vel_y",
            "vel_z": "/target_vel_z",
            "ang_z": "/target_ang_z",
        }

        for key, topic_name in target_topics.items():
            self.create_subscription(
                Float32,
                topic_name,
                # partialを使って、どのターゲットを更新するかをコールバックに伝える
                partial(self.target_callback, target_key=key),
                10,
            )

        self.get_logger().info("Multi-dimensional subscriber node has been started.")

    def target_callback(self, msg: Float32, target_key: str):
        with self.lock:
            self.latest_targets[target_key] = msg.data

    def odom_callback(self, msg: Odometry):
        timestamp = time.time()
        with self.lock:
            # 現在の値をタプルとして取得
            current_values = (
                # msg.pose.position.x,
                # msg.pose.position.y,
                msg.pose.position.z_altitude,
                msg.pose.orientation.z,
                # msg.twist.linear.x,/
                # msg.twist.linear.y,
                msg.twist.linear.z_altitude,
                msg.twist.angular.z,
            )
            # 現在の目標値をタプルとして取得
            current_targets = (
                self.latest_targets["vel_x"],
                self.latest_targets["vel_y"],
                self.latest_targets["vel_z"],
                self.latest_targets["ang_z"],
            )
            # (タイムスタンプ, 値のタプル, 目標値のタプル) の形式でキューに追加
            self.data_queue.append((timestamp, current_values, current_targets))

    def get_data_points(self):
        points = []
        with self.lock:
            while self.data_queue:
                points.append(self.data_queue.popleft())
        return points


def main(args=None):
    rclpy.init(args=args)
    ros_node = MultiDimSubscriber()

    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    viewer = MultiGraphViewer(ros_node=ros_node)
    viewer.show()

    ret = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
