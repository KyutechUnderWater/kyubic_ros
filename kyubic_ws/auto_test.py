#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import time
from localization_msgs.msg import Odometry
from driver_msgs.msg import PowerState, IMU
from joy_common_msgs.msg import Joy, Buttons, Stick


class AutoTestNode(Node):
    def __init__(self):
        super().__init__("auto_test_node")

        self.pub_odom = self.create_publisher(Odometry, "/localization/gnss/odom", 10)
        self.pub_power = self.create_publisher(
            PowerState, "/driver/logic_distro_rp2040_driver/power_state", 10
        )
        self.pub_imu = self.create_publisher(IMU, "/driver/imu_driver/imu", 10)
        self.pub_joy = self.create_publisher(Joy, "/joy_common/joy_common", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.origin_lat = 33.900000
        self.origin_lon = 130.900000

        self.start_time = time.time()
        self.phase = 0

    def latlon_to_xy(self, lat, lon):
        R = 6378137.0
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)

        x = R * (lat_rad - origin_lat_rad)
        y = R * math.cos(origin_lat_rad) * (lon_rad - origin_lon_rad)
        return x, y

    def timer_callback(self):
        elapsed = time.time() - self.start_time

        # 1. 常時送信：BT解除用センサー情報
        power_msg = PowerState()
        power_msg.log_voltage = 15.0
        power_msg.act_voltage = 15.0
        self.pub_power.publish(power_msg)

        imu_msg = IMU()
        imu_msg.status.id = 0
        self.pub_imu.publish(imu_msg)

        # 2. 2秒後：Joyコンの「cross（Autoボタン）」を押す
        if 2.0 < elapsed < 2.5:
            joy_msg = Joy()
            joy_msg.buttons.cross = True
            self.pub_joy.publish(joy_msg)
            if self.phase == 0:
                print(">>> Phase 1: Published Joy 'cross' button to start Auto mode")
                self.phase = 1

        # 3. 5秒後以降：仮想位置を送信して経路計画をテスト
        odom_msg = Odometry()
        odom_msg.pose.orientation.z = 0.0

        if 5.0 < elapsed < 10.0:
            # 第一ウェイポイントへ
            x, y = self.latlon_to_xy(33.900100, 130.900100)
            odom_msg.pose.position.x = x
            odom_msg.pose.position.y = y
            odom_msg.pose.position.z_depth = 1.0
            if self.phase == 1:
                print(">>> Phase 2: Arriving at Waypoint 1 (33.900100, 130.900100, Depth: 1.0)")
                self.phase = 2

        elif elapsed >= 10.0:
            # 第二ウェイポイントへ
            x, y = self.latlon_to_xy(33.900200, 130.900200)
            odom_msg.pose.position.x = x
            odom_msg.pose.position.y = y
            odom_msg.pose.position.z_depth = -1.5
            if self.phase == 2:
                print(">>> Phase 3: Arriving at Waypoint 2 (33.900200, 130.900200, Depth: -1.5)")
                self.phase = 3

        if elapsed > 5.0:
            self.pub_odom.publish(odom_msg)

        if elapsed > 15.0:
            print(">>> Test sequence complete. Exiting.")
            raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    node = AutoTestNode()
    print("Starting Automated Test Sequence...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
