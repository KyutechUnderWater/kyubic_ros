#!/usr/bin/env python3
"""Combine the real depth/IMU streams for the Day 4 preview plot."""

from localization_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class SensorPreviewNode(Node):
    def __init__(self):
        super().__init__("educational_pid_sensor_preview")
        self.depth_msg = None
        self.imu_msg = None
        self.pub_preview = self.create_publisher(Odometry, "preview_odom", 10)
        self.sub_depth = self.create_subscription(
            Odometry,
            "depth_odom",
            self.depth_callback,
            10,
        )
        self.sub_imu = self.create_subscription(
            Odometry,
            "imu_odom",
            self.imu_callback,
            10,
        )
        self.timer = self.create_timer(0.05, self.publish_preview)

    def depth_callback(self, msg):
        self.depth_msg = msg

    def imu_callback(self, msg):
        self.imu_msg = msg

    def publish_preview(self):
        if self.depth_msg is None or self.imu_msg is None:
            return

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.z_depth = self.depth_msg.pose.position.z_depth
        msg.twist.linear.z_depth = self.depth_msg.twist.linear.z_depth
        msg.pose.orientation.z = self.imu_msg.pose.orientation.z
        msg.twist.angular.z = self.imu_msg.twist.angular.z
        msg.status.depth.id = self.depth_msg.status.depth.id
        msg.status.imu.id = self.imu_msg.status.imu.id
        self.pub_preview.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorPreviewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
