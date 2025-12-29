import sys
import threading
import collections
import time
from functools import partial

# ROS2関連
import rclpy
from rclpy.node import Node
from localization_msgs.msg import Odometry
from p_pid_controller_msgs.msg import Targets

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
    QCheckBox,
    QGridLayout,
)
from PyQt5.QtCore import QTimer, Qt

# PyQtGraphの基本設定
import pyqtgraph as pg

pg.setConfigOption("background", "w")
pg.setConfigOption("foreground", "k")

# プロットするデータの最大保持数
MAX_DATA_POINTS = 3000  # subscriber 5Hz -> buffer 10m
INITIAL_WIDTH = 20.0  # [s]


class PlotUnitWidget(QWidget):
    """
    一つのグラフと、その設定UIをまとめたウィジェットクラス
    """

    def __init__(
        self,
        title,
        y1_label,
        y2_label,
        unit1,
        unit2,
        min1_y,
        max1_y,
        min2_y,
        max2_y,
        invert_y=False,
        parent=None,
    ):
        super(PlotUnitWidget, self).__init__(parent)
        layout = QVBoxLayout()
        self.setLayout(layout)

        # --- Y軸範囲設定UI ---
        yrange_layout = QHBoxLayout()
        yrange_layout.addWidget(QLabel(f"{title}"))
        yrange_layout.addStretch(1)

        yrange_layout.addWidget(QLabel("min1:"))
        self.min1_y_input = QLineEdit(f"{min1_y}")
        self.min1_y_input.setFixedWidth(40)
        yrange_layout.addWidget(self.min1_y_input)

        yrange_layout.addWidget(QLabel("max1:"))
        self.max1_y_input = QLineEdit(f"{max1_y}")
        self.max1_y_input.setFixedWidth(40)
        yrange_layout.addWidget(self.max1_y_input)

        yrange_layout.addWidget(QLabel("min2:"))
        self.min2_y_input = QLineEdit(f"{min2_y}")
        self.min2_y_input.setFixedWidth(40)
        yrange_layout.addWidget(self.min2_y_input)

        yrange_layout.addWidget(QLabel("max2:"))
        self.max2_y_input = QLineEdit(f"{max2_y}")
        self.max2_y_input.setFixedWidth(40)
        yrange_layout.addWidget(self.max2_y_input)

        set_range_button = QPushButton("Set range")
        set_range_button.setDefault(True)
        set_range_button.clicked.connect(self.update_yrange)
        yrange_layout.addWidget(set_range_button)

        # --- グラフウィジェット ---
        # 1つ目のデータ
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setMouseEnabled(x=True, y=False)
        self.plot_widget.setLabel("left", y1_label, units=unit1)
        self.plot_widget.setLabel("bottom", "Time", units="s")
        self.plot_widget.getViewBox().invertY(invert_y)
        self.plot_widget.showGrid(x=True, y=True, alpha=0.2)
        self.plot_widget.addLegend()
        self.plot_widget.mouseDoubleClickEvent = self.auto_scale_y

        # ２つ目のデータ
        self.plot_widget2 = pg.ViewBox()
        self.plot_widget2.setMouseEnabled(y=False)
        self.plot_widget2.enableAutoRange(axis="x", enable=True)
        self.plot_widget2.invertY(invert_y)
        self.plot_widget.scene().addItem(self.plot_widget2)
        self.plot_widget.showAxis("right")
        self.plot_widget.setLabel("right", y2_label, units=unit2)
        self.plot_widget.getAxis("right").linkToView(self.plot_widget2)
        self.plot_widget.getAxis("right").setGrid(False)

        # １つ目のデータのグラフのサイズ変化を検知し，２つ目のデータにも適応する
        self.plot_widget.plotItem.vb.sigResized.connect(self.update_views)
        self.plot_widget2.setXLink(self.plot_widget)

        # グラフのオブジェクト生成
        self.target_curve = self.plot_widget.plot(
            pen=pg.mkPen("r", width=2, style=pg.QtCore.Qt.DashLine), name="Target"
        )
        self.value1_curve = self.plot_widget.plot(pen=pg.mkPen("b", width=2), name="Position")
        self.value2_curve = pg.PlotCurveItem(
            pen=pg.mkPen(color=(0, 255, 0, 90), width=2), name="Velocity"
        )
        self.plot_widget2.addItem(self.value2_curve)
        self.plot_widget.plotItem.legend.addItem(self.value2_curve, "Velocity")

        layout.addLayout(yrange_layout)
        layout.addWidget(self.plot_widget)

        self.init = False

    def setData(self, time_data, value1_data, value2_data, target_data):
        self.value1_curve.setData(time_data, value1_data)
        self.value2_curve.setData(time_data, value2_data)
        self.target_curve.setData(time_data, target_data)

        # 最初に１度だけ，y軸レンジを初期値で設定
        if not self.init:
            self.init = True
            self.update_yrange()

    def update_yrange(self):
        try:
            min1_y = float(self.min1_y_input.text())
            max1_y = float(self.max1_y_input.text())
            min2_y = float(self.min2_y_input.text())
            max2_y = float(self.max2_y_input.text())
            if min1_y < max1_y:
                self.plot_widget.setYRange(min1_y, max1_y)
            if min2_y < max2_y:
                self.plot_widget2.setYRange(min2_y, max2_y)
        except ValueError:
            pass  # 不正な値は無視

    def update_views(self):
        # メインのViewBoxのサイズ変更に合わせて、2つ目のViewBoxとAxisItemの位置を調整
        self.plot_widget2.setGeometry(self.plot_widget.plotItem.vb.sceneBoundingRect())

    def auto_scale_y(self, event):
        """
        グラフをダブルクリックしたときにY軸のスケールを自動調整する
        """
        self.plot_widget.enableAutoRange(axis="y")
        self.plot_widget2.enableAutoRange(axis="y")


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
        self.layout = QVBoxLayout()
        grid_layout = QGridLayout()
        self.setCentralWidget(central_widget)
        central_widget.setLayout(self.layout)
        self.layout.addLayout(grid_layout)

        self.last_data_time = 0.0

        # --- 6つのグラフエリアを作成 ---
        plot_cfgs = [
            {
                "title": "Pose X",
                "unit1": "m",
                "unit2": "m/s",
                "min1_y": -3,
                "max1_y": 3,
                "min2_y": -0.6,
                "max2_y": 0.6,
            },
            {
                "title": "Pose Y",
                "unit1": "m",
                "unit2": "m/s",
                "min1_y": -3,
                "max1_y": 3,
                "min2_y": -0.6,
                "max2_y": 0.6,
            },
            {
                "title": "Pose Z (Depth)",
                "unit1": "m",
                "unit2": "m/s",
                "min1_y": 0,
                "max1_y": 1,
                "min2_y": -0.3,
                "max2_y": 0.3,
                "invert_y": True,
            },
            {
                "title": "Pose Z (Altitude)",
                "unit1": "m",
                "unit2": "m/s",
                "min1_y": 0,
                "max1_y": 1,
                "min2_y": -0.3,
                "max2_y": 0.3,
            },
            {
                "title": "Angle X",
                "unit1": "deg",
                "unit2": "deg/s",
                "min1_y": -30,
                "max1_y": 30,
                "min2_y": -30,
                "max2_y": 30,
            },
            {
                "title": "Angle Z",
                "unit1": "deg",
                "unit2": "deg/s",
                "min1_y": -180,
                "max1_y": 180,
                "min2_y": -50,
                "max2_y": 50,
            },
        ]

        self.plot_units = []
        for i, cfg in enumerate(plot_cfgs):
            plot_unit = PlotUnitWidget(y1_label="Position", y2_label="Velocity", **cfg)
            self.plot_units.append(plot_unit)

            # X軸の同期設定
            if i > 0:
                plot_unit.plot_widget.setXLink(self.plot_units[0].plot_widget)

            # グリッドレイアウトに配置 (2x3)
            grid_layout.addWidget(plot_unit, i % 2, i // 2)

        self.n_graph = len(self.plot_units)

        # --- tool area ---
        self.foot_layout = QHBoxLayout()
        self.layout.addLayout(self.foot_layout)

        # 表示するデータの選択
        self.checkbox_target = QCheckBox("Show Target")
        self.checkbox_target.setFixedWidth(100)
        self.checkbox_target.setChecked(True)  # 初期状態は表示
        self.checkbox_target.stateChanged.connect(lambda state: self.setPlotVisible(0, state))
        self.foot_layout.addWidget(self.checkbox_target)

        self.checkbox_curve1 = QCheckBox("Show Position")
        self.checkbox_curve1.setFixedWidth(110)
        self.checkbox_curve1.setChecked(True)  # 初期状態は表示
        self.checkbox_curve1.stateChanged.connect(lambda state: self.setPlotVisible(1, state))
        self.foot_layout.addWidget(self.checkbox_curve1)

        self.checkbox_curve2 = pg.QtWidgets.QCheckBox("Show Velocity")
        self.checkbox_curve2.setFixedWidth(110)
        self.checkbox_curve2.setChecked(True)  # 初期状態は表示
        self.checkbox_curve2.stateChanged.connect(lambda state: self.setPlotVisible(2, state))
        self.foot_layout.addWidget(self.checkbox_curve2)

        # リアルタイム表示に戻すボタン
        self.reset_button_state = True
        self.reset_button = QPushButton("Return to real-time plot")
        self.reset_button.clicked.connect(self.reset)
        self.foot_layout.addWidget(self.reset_button)

        # --- データ保持用バッファ ---
        self.time_data = collections.deque(maxlen=MAX_DATA_POINTS)
        # 0:pos_x, 1:pos_y, 2:pos_z_depth, 3:pos_z_alt, 4:orient_x, 5:orient_z,
        # 6:vel_x, 7:vel_y, 8:vel_z_depth, 9:vel_z_alt, 3:ang_x, 3:ang_z
        self.value_data = [
            collections.deque(maxlen=MAX_DATA_POINTS) for _ in range(self.n_graph * 2)
        ]
        self.target_data = [
            collections.deque(maxlen=MAX_DATA_POINTS) for _ in range(self.n_graph * 2)
        ]

        # --- グラフ更新用タイマー ---
        self.timer = QTimer()
        self.timer.setInterval(50)  # 50msごとに更新 (20Hz)
        self.timer.timeout.connect(self.update_plots)
        self.timer.start()

    def setPlotVisible(self, curve, state):
        if curve == 0:
            for plot in self.plot_units:
                plot.target_curve.setVisible(state == Qt.Checked)
        elif curve == 1:
            for plot in self.plot_units:
                plot.value1_curve.setVisible(state == Qt.Checked)
        elif curve == 2:
            for plot in self.plot_units:
                plot.value2_curve.setVisible(state == Qt.Checked)

    def reset(self):
        self.reset_button_state = True
        self.plot_units[0].plot_widget.enableAutoRange(axis="x", enable=False)
        self.plot_units[0].plot_widget2.enableAutoRange(axis="x", enable=False)

    def update_plots(self):
        new_data_points = self.ros_node.get_data_points()

        if not new_data_points:
            return

        for data_tuple in new_data_points:
            timestamp, values, targets = data_tuple
            self.time_data.append(timestamp - self.start_time)
            for i in range(self.n_graph * 2):
                self.value_data[i].append(values[i])
                self.target_data[i].append(targets[i])

        # 描画データのセット
        time_list = list(self.time_data)
        for i in range(self.n_graph):
            self.plot_units[i].setData(
                time_list,
                list(self.value_data[i]),
                list(self.value_data[i + self.n_graph]),
                list(self.target_data[i]),
            )

        latest_time = time_list[-1]

        # 現在のビュー情報を取得
        vb = self.plot_units[0].plot_widget.getViewBox()
        view_min, view_max = vb.viewRange()[0]
        current_width = view_max - view_min

        # 追従判定 (前回のデータ時刻と比較)
        is_at_edge = (self.last_data_time - view_max) < 1.0

        # リセットボタン押下時（および初期起動時）幅を20秒に強制，最新へ移動
        if self.reset_button_state:
            self.reset_button_state = False
            self.plot_units[0].plot_widget.setXRange(
                latest_time - INITIAL_WIDTH, latest_time, padding=0
            )
        elif is_at_edge:
            self.plot_units[0].plot_widget.setXRange(
                latest_time - current_width, latest_time, padding=0
            )

        self.last_data_time = latest_time


class MultiDimSubscriber(Node):
    """
    Odometryと複数の目標値トピックを購読するROS2ノードクラス
    """

    def __init__(self):
        super().__init__("rt_pose_plotter")

        self.data_queue = collections.deque(maxlen=MAX_DATA_POINTS)
        self.lock = threading.Lock()

        # 4つの目標値を辞書で管理
        self.latest_targets: Targets = Targets()

        # サブスクライバの定義
        self.odom_subscription = self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        self.create_subscription(
            Targets,
            "targets",
            # partialを使って、どのターゲットを更新するかをコールバックに伝える
            partial(self.target_callback),
            10,
        )

        self.get_logger().info("Multi-dimensional subscriber node has been started.")

    def target_callback(self, msg: Targets):
        with self.lock:
            self.latest_targets = msg

    def odom_callback(self, msg: Odometry):
        timestamp = time.time()
        with self.lock:
            # 現在の値をタプルとして取得
            current_values = (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z_depth,
                msg.pose.position.z_altitude,
                msg.pose.orientation.x,
                msg.pose.orientation.z,
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z_depth,
                msg.twist.linear.z_altitude,
                msg.twist.angular.x,
                msg.twist.angular.z,
            )

            # 現在の目標値をタプルとして取得
            current_targets = (
                self.latest_targets.pose.x,
                self.latest_targets.pose.y,
                self.latest_targets.pose.z,
                self.latest_targets.pose.z,
                self.latest_targets.pose.roll,
                self.latest_targets.pose.yaw,
                self.latest_targets.twist.x,
                self.latest_targets.twist.y,
                self.latest_targets.twist.z,
                self.latest_targets.twist.z,
                self.latest_targets.twist.roll,
                self.latest_targets.twist.yaw,
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
