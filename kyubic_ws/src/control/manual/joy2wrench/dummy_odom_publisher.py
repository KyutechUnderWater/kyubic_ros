#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

class DummyOdomPublisher(Node):
    def __init__(self):
        super().__init__('dummy_odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 0.1秒ごとに配信
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.get_logger().info('Dummy Odometry Publisher has been started.')

    def timer_callback(self):
        msg = Odometry()

        current_time = self.get_clock().now().to_msg()
        msg.header.stamp = current_time
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # 簡単な動きをシミュレート（ゆっくり回転）
        self.theta += 0.01

        # クォータニオンの計算
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = q

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()