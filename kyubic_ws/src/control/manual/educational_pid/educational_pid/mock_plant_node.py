#!/usr/bin/env python3
"""Small depth/yaw plant for PC-only closed-loop exercises."""

import rclpy
from common_msgs.msg import Status
from geometry_msgs.msg import WrenchStamped
from localization_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger


class MockPlantNode(Node):
    def __init__(self):
        super().__init__("educational_pid_mock_plant")
        self.declare_parameter("period", 0.05)
        self.declare_parameter("mass", 12.0)
        self.declare_parameter("depth_drag", 1.4)
        self.declare_parameter("depth_disturbance", -0.05)
        self.declare_parameter("yaw_accel_per_torque", 20.0)
        self.declare_parameter("yaw_drag", 1.5)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("initial_depth", 0.0)
        self.declare_parameter("initial_yaw", 0.0)

        self.pub_depth = self.create_publisher(Odometry, "depth_odom", 10)
        self.pub_imu = self.create_publisher(Odometry, "imu_odom", 10)
        self.pub_localization_odom = self.create_publisher(
            Odometry,
            "localization_odom",
            10,
        )
        self.sub_wrench = self.create_subscription(
            WrenchStamped,
            "robot_force",
            self.wrench_callback,
            10,
        )
        self.srv_reset = self.create_service(
            Trigger,
            "/educational_pid/mock_reset",
            self.reset_callback,
        )

        self.force_z = 0.0
        self.torque_z = 0.0
        self.last_command_time = None
        self.reset_state()
        self.timer = self.create_timer(
            self.get_parameter("period").value,
            self.update,
        )
        self.get_logger().info("PC-only mock plant started; no hardware topics are used")

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def reset_state(self):
        self.depth = self.get_parameter("initial_depth").value
        self.depth_velocity = 0.0
        self.yaw = self.get_parameter("initial_yaw").value
        self.yaw_rate = 0.0
        self.force_z = 0.0
        self.torque_z = 0.0
        self.last_command_time = None

    def reset_callback(self, _request, response):
        self.reset_state()
        response.success = True
        response.message = "mock plant reset"
        return response

    def wrench_callback(self, msg):
        self.force_z = msg.wrench.force.z
        self.torque_z = msg.wrench.torque.z
        self.last_command_time = self._now_sec()

    def update(self):
        now = self._now_sec()
        timeout = self.get_parameter("command_timeout").value
        if self.last_command_time is None or now - self.last_command_time > timeout:
            self.force_z = 0.0
            self.torque_z = 0.0

        dt = self.get_parameter("period").value
        mass = self.get_parameter("mass").value
        depth_drag = self.get_parameter("depth_drag").value
        disturbance = self.get_parameter("depth_disturbance").value
        depth_accel = self.force_z / mass + disturbance - depth_drag * self.depth_velocity
        self.depth_velocity += depth_accel * dt
        self.depth += self.depth_velocity * dt
        if self.depth < 0.0:
            self.depth = 0.0
            self.depth_velocity = max(0.0, self.depth_velocity)

        yaw_gain = self.get_parameter("yaw_accel_per_torque").value
        yaw_drag = self.get_parameter("yaw_drag").value
        yaw_accel = self.torque_z * yaw_gain - yaw_drag * self.yaw_rate
        self.yaw_rate += yaw_accel * dt
        self.yaw += self.yaw_rate * dt
        self.yaw = (self.yaw + 180.0) % 360.0 - 180.0

        stamp = self.get_clock().now().to_msg()
        depth_msg = Odometry()
        depth_msg.header.stamp = stamp
        depth_msg.status.depth.id = Status.NORMAL
        depth_msg.pose.position.z_depth = self.depth
        depth_msg.twist.linear.z_depth = self.depth_velocity
        self.pub_depth.publish(depth_msg)

        imu_msg = Odometry()
        imu_msg.header.stamp = stamp
        imu_msg.status.imu.id = Status.NORMAL
        imu_msg.pose.orientation.z = self.yaw
        imu_msg.twist.angular.z = self.yaw_rate
        self.pub_imu.publish(imu_msg)

        # rt_pose_plotter用に、実機と同じ形式の統合OdometryもPublishする。
        # 新しく作ったメッセージの未設定数値フィールドは0.0になるため、
        # poseの深度・Yaw以外は0.0のまま送信される。
        localization_odom_msg = Odometry()
        localization_odom_msg.header.stamp = stamp
        localization_odom_msg.header.frame_id = "odom"
        localization_odom_msg.status.depth.id = Status.NORMAL
        localization_odom_msg.status.imu.id = Status.NORMAL
        localization_odom_msg.status.dvl.id = Status.NORMAL
        localization_odom_msg.pose.position.z_depth = self.depth
        localization_odom_msg.pose.orientation.z = self.yaw
        self.pub_localization_odom.publish(localization_odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockPlantNode()
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
