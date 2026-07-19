#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from localization_msgs.msg import Odometry
from localization_msgs.srv import Reset
from driver_msgs.msg import PowerState, IMU


class DummyPublisher(Node):
    def __init__(self):
        super().__init__("dummy_gnss_publisher")

        # 経路計画用の仮想自己位置（メートル変換後）
        self.pub_odom = self.create_publisher(Odometry, "/localization/gnss/odom", 10)

        # Behavior Treeの待機ブロックを解除するための仮想センサー情報
        # ※launchファイルでのリマッピングに合わせたトピック名にしています
        self.pub_power = self.create_publisher(
            PowerState, "/driver/logic_distro_rp2040_driver/power_state", 10
        )
        self.pub_imu = self.create_publisher(IMU, "/driver/imu_driver/imu", 10)

        # BTのAutoモード開始時に呼ばれるリセットサービスをモック（偽装）する
        self.srv_reset = self.create_service(Reset, "/localization/reset", self.reset_callback)

        self.timer = self.create_timer(0.1, self.timer_callback)

        # YAMLで設定した基準原点
        self.origin_lat = 33.900000
        self.origin_lon = 130.900000

    def latlon_to_xy(self, lat, lon):
        R = 6378137.0
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)

        x = R * (lat_rad - origin_lat_rad)
        y = R * math.cos(origin_lat_rad) * (lon_rad - origin_lon_rad)
        return x, y

    def reset_callback(self, request, response):
        # Autoモードに入った時にBTから呼ばれるリセットを「成功」として返す
        self.get_logger().info("Received dummy reset request from BT")
        response.success = True
        response.message = "Dummy reset successful"
        return response

    def timer_callback(self):
        # --- 1. BT解除用のバッテリー情報 ---
        power_msg = PowerState()
        power_msg.log_voltage = 15.0  # 13.2V以上でOK
        power_msg.act_voltage = 15.0
        self.pub_power.publish(power_msg)

        # --- 2. BT解除用のセンサー正常情報 ---
        imu_msg = IMU()
        imu_msg.status.id = 0  # 0=OK
        self.pub_imu.publish(imu_msg)

        # --- 3. 経路計画用の仮想現在地 ---
        odom_msg = Odometry()

        # ==========================================
        # ★ここでテストしたい現在地を入力してください！★
        current_lat = 33.900100  # 仮想の現在地（緯度）
        current_lon = 130.900100  # 仮想の現在地（経度）
        current_z = -1.0  # 仮想の現在地（深度 / 高度）

        # ちなみに第2ウェイポイントに進ませたい場合は以下をコメント解除してください
        # current_lat = 33.900200
        # current_lon = 130.900200
        # current_z = -1.5
        # ==========================================

        x, y = self.latlon_to_xy(current_lat, current_lon)
        odom_msg.pose.position.x = x
        odom_msg.pose.position.y = y
        # Z_MODE_DEPTH(0), Z_MODE_ALTITUDE(1) のどちらでも到達判定をパスできるよう両方に入れます
        odom_msg.pose.position.z_depth = current_z
        odom_msg.pose.position.z_altitude = current_z
        odom_msg.pose.orientation.z = 0.0

        self.pub_odom.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyPublisher()
    print("安全な仮想テストモード: /localization/gnss/odom および BT解除用データを送信中...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    import sys

    main(sys.argv)
