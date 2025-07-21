import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Vector3  # Twist.angular用
import pandas as pd
import numpy as np
import math  # for radians conversion and interpolation

# カスタムメッセージ型のインポート
try:
    from localization_msgs.msg import Odometry, Point, EulerAngle, Pose, Twist
except ImportError:
    print(
        "Error: Could not import custom messages. "
        "Please ensure 'localization_msgs' package is built in your ROS2 workspace."
    )
    print(
        "Example: from your_package_name.msg import Odometry, Point, EulerAngle, Pose, Twist"
    )
    sys.exit(1)

import sys
import os


class OdometryPublisher(Node):
    def __init__(self, csv_file_path, publish_interval_sec=0.1):
        super().__init__("odometry_publisher")

        self.publish_interval_sec = publish_interval_sec  # パブリッシュ間隔（秒）

        # Publisherの作成
        self.publisher_ = self.create_publisher(Odometry, "localization/odom", 10)

        # タイマーの作成
        self.timer = self.create_timer(self.publish_interval_sec, self.publish_odometry)
        self.get_logger().info(
            f"Odometry publisher node started with {publish_interval_sec}s interval."
        )

        # CSVファイルの読み込み
        self.csv_file_path = csv_file_path
        self.df = self.load_csv_data()

        self.total_rows = len(self.df)
        if self.total_rows < 2:  # 少なくとも2点ないと補間できない
            self.get_logger().error(
                "CSV file must contain at least 2 data points for interpolation."
            )
            rclpy.shutdown()
            return

        # 補間関連の変数
        self.current_segment_start_index = (
            0  # 現在補間中のセグメントの開始点（CSVの行インデックス）
        )
        self.interpolation_step = (
            0  # 現在のセグメント内での補間ステップ数 (0.0 から 1.0 まで)
        )
        self.steps_per_segment = (
            1.0 / self.publish_interval_sec
        )  # 1秒あたり何ステップ進むか（CSVが1秒ごとのキーポイントと仮定）

        # 最初のセグメントのデータをセット
        self.start_point = self.df.iloc[self.current_segment_start_index]
        self.end_point = self.df.iloc[self.current_segment_start_index + 1]

        # 初期速度と角速度
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_linear_z = 0.0  # z_depthに対応
        self.current_angular_roll = 0.0  # オイラー角のx
        self.current_angular_yaw = 0.0  # オイラー角のz

    def load_csv_data(self):
        try:
            df = pd.read_csv(self.csv_file_path)
            self.get_logger().info(
                f"Successfully loaded CSV from: {self.csv_file_path}"
            )
            self.get_logger().info(f"Columns: {df.columns.tolist()}")

            required_columns = [
                "catmull_x",
                "catmull_y",
                "catmull_z",
                "catmull_roll",
                "catmull_yaw",
            ]
            if not all(col in df.columns for col in required_columns):
                self.get_logger().error(
                    f"CSV is missing one or more required columns: {required_columns}"
                )
                rclpy.shutdown()

            # 必要な列だけを抽出し、NaNを0に変換
            df_filtered = (
                df[required_columns].fillna(0.0).astype(float)
            )  # float型に明示的に変換

            return df_filtered

        except FileNotFoundError:
            self.get_logger().error(f"CSV file not found at: {self.csv_file_path}")
            rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Error loading CSV: {e}")
            rclpy.shutdown()
        return pd.DataFrame()

    def linear_interpolation(self, start_val, end_val, t):
        """t (0.0-1.0) に基づいて線形補間を行う"""
        return start_val + (end_val - start_val) * t

    def publish_odometry(self):
        # 補間ステップを更新
        # 1秒を `steps_per_segment` 回に分割しているので、1ステップは 1/steps_per_segment
        t_interp = self.interpolation_step / self.steps_per_segment

        # 現在のセグメントの終了点に到達したかチェック
        if t_interp >= 1.0:
            # 次のセグメントへ移動
            self.current_segment_start_index += 1
            self.interpolation_step = 0  # 新しいセグメントの最初から

            if self.current_segment_start_index >= self.total_rows - 1:
                self.get_logger().info(
                    "Finished publishing all interpolated odometry data from CSV."
                )
                self.timer.cancel()  # タイマーを停止
                rclpy.shutdown()  # ノードを終了
                return

            self.start_point = self.df.iloc[self.current_segment_start_index]
            self.end_point = self.df.iloc[self.current_segment_start_index + 1]
            t_interp = 0.0  # リセットされたので再度0から開始

        # 線形補間された位置と姿勢を計算
        interp_x = self.linear_interpolation(
            self.start_point["catmull_x"], self.end_point["catmull_x"], t_interp
        )
        interp_y = self.linear_interpolation(
            self.start_point["catmull_y"], self.end_point["catmull_y"], t_interp
        )
        interp_z = self.linear_interpolation(
            self.start_point["catmull_z"], self.end_point["catmull_z"], t_interp
        )
        interp_roll = self.linear_interpolation(
            self.start_point["catmull_roll"], self.end_point["catmull_roll"], t_interp
        )
        interp_yaw = self.linear_interpolation(
            self.start_point["catmull_yaw"], self.end_point["catmull_yaw"], t_interp
        )

        # 速度と角速度の計算
        # 簡易的な速度計算: 現在と次のステップの差分を publish_interval_sec で割る
        # これを厳密にやるには、前回の位置を保持しておく必要があるが、ここではLERPの差分を利用
        next_x = self.linear_interpolation(
            self.start_point["catmull_x"],
            self.end_point["catmull_x"],
            t_interp + (1.0 / self.steps_per_segment),
        )
        next_y = self.linear_interpolation(
            self.start_point["catmull_y"],
            self.end_point["catmull_y"],
            t_interp + (1.0 / self.steps_per_segment),
        )
        next_z = self.linear_interpolation(
            self.start_point["catmull_z"],
            self.end_point["catmull_z"],
            t_interp + (1.0 / self.steps_per_segment),
        )

        # 現在のステップからの移動距離を時間で割る
        self.current_linear_x = (next_x - interp_x) / self.publish_interval_sec
        self.current_linear_y = (next_y - interp_y) / self.publish_interval_sec
        self.current_linear_z = (next_z - interp_z) / self.publish_interval_sec

        # 角速度も同様に計算
        next_roll = self.linear_interpolation(
            self.start_point["catmull_roll"],
            self.end_point["catmull_roll"],
            t_interp + (1.0 / self.steps_per_segment),
        )
        next_yaw = self.linear_interpolation(
            self.start_point["catmull_yaw"],
            self.end_point["catmull_yaw"],
            t_interp + (1.0 / self.steps_per_segment),
        )

        # 角度の差分を考慮した角速度（周期性のある角度の場合、-piからpiの範囲での最短経路を考慮）
        # yawの差分計算
        delta_yaw = next_yaw - interp_yaw
        # -pi から pi の範囲に正規化 (オプション: 大きな角度変化の場合に必要)
        # delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))
        self.current_angular_yaw = delta_yaw / self.publish_interval_sec

        # rollの差分計算 (同上)
        delta_roll = next_roll - interp_roll
        self.current_angular_roll = delta_roll / self.publish_interval_sec

        # Odometryメッセージの作成
        odom_msg = Odometry()

        # Headerの設定
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom_frame"

        # Statusの設定 (ここでは常にNORMALとする)
        odom_msg.status = Odometry.STATUS_NORMAL

        # Poseの設定
        odom_msg.pose = Pose()

        # Positionの設定
        odom_msg.pose.position = Point()
        odom_msg.pose.position.x = interp_x
        odom_msg.pose.position.y = interp_y
        odom_msg.pose.position.z_depth = interp_z
        odom_msg.pose.position.z_altitude = 0.0  # CSVにないので0.0
        odom_msg.pose.position.z_altitude_estimate = 0.0  # CSVにないので0.0

        # Orientationの設定
        odom_msg.pose.orientation = EulerAngle()
        odom_msg.pose.orientation.x = interp_roll
        odom_msg.pose.orientation.y = 0.0  # CSVにないので0.0
        odom_msg.pose.orientation.z = interp_yaw

        # Twistの設定 (計算した速度・角速度を設定)
        odom_msg.twist = Twist()
        odom_msg.twist.linear = Point()
        odom_msg.twist.linear.x = self.current_linear_x
        odom_msg.twist.linear.y = self.current_linear_y
        odom_msg.twist.linear.z_depth = self.current_linear_z
        odom_msg.twist.linear.z_altitude = 0.0  # CSVにないので0.0
        odom_msg.twist.linear.z_altitude_estimate = 0.0  # CSVにないので0.0

        odom_msg.twist.angular = Vector3()
        odom_msg.twist.angular.x = self.current_angular_roll  # rollの角速度
        odom_msg.twist.angular.y = 0.0  # CSVにないので0.0
        odom_msg.twist.angular.z = self.current_angular_yaw  # yawの角速度

        # メッセージをパブリッシュ
        self.publisher_.publish(odom_msg)
        self.get_logger().info(
            f"Publishing LERP Odometry (Segment {self.current_segment_start_index}, Step {self.interpolation_step + 1}/{int(self.steps_per_segment)}): "
            f"Pos:({interp_x:.2f}, {interp_y:.2f}, {interp_z:.2f}), "
            f"Ori:({interp_roll:.2f}, {interp_yaw:.2f}), "
            f"LinVel:({self.current_linear_x:.2f}, {self.current_linear_y:.2f}), "
            f"AngVel:({self.current_angular_roll:.2f}, {self.current_angular_yaw:.2f})"
        )

        self.interpolation_step += 1


def main(args=None):
    rclpy.init(args=args)

    script_dir = os.path.dirname(__file__)
    csv_file_path = os.path.join(
        script_dir, "data.csv"
    )  # スクリプトと同じディレクトリのdata.csvを想定

    # 0.1秒間隔でパブリッシュする
    odometry_publisher = OdometryPublisher(csv_file_path, publish_interval_sec=0.1)

    try:
        rclpy.spin(odometry_publisher)
    except KeyboardInterrupt:
        odometry_publisher.get_logger().info("Node stopped by user.")
    finally:
        odometry_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
