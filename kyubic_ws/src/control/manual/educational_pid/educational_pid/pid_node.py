#!/usr/bin/env python3
"""ROS 2 interfaces and executable for the educational PID controller."""

import rclpy
from geometry_msgs.msg import WrenchStamped
from joy_common_msgs.msg import Joy
from localization_msgs.msg import Odometry
from p_pid_controller_msgs.msg import Targets
from p_pid_controller_msgs.srv import SetTargets
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger

from educational_pid.controller_node import EducationalPidControllerBase


class EducationalPidNode(EducationalPidControllerBase):
    def create_interfaces(self):
        self.pub_wrench = self.create_publisher(WrenchStamped, "robot_force", 10)
        self.pub_targets = self.create_publisher(Targets, "targets", 10)
        self.pub_integral_depth = self.create_publisher(Float64, "integral_depth", 10)
        self.pub_integral_yaw = self.create_publisher(Float64, "integral_yaw", 10)
        self.pub_error_depth = self.create_publisher(Float64, "error_depth", 10)
        self.pub_error_yaw = self.create_publisher(Float64, "error_yaw", 10)
        self.sub_joy = self.create_subscription(Joy, "joy_common", self.joy_callback, 10)
        self.sub_depth = self.create_subscription(Odometry, "depth_odom", self.depth_callback, 10)
        self.sub_imu = self.create_subscription(Odometry, "imu_odom", self.imu_callback, 10)
        self.srv_enable = self.create_service(
            SetBool, "/educational_pid/enable", self.enable_callback
        )
        self.srv_reset = self.create_service(
            Trigger, "/educational_pid/reset_pid", self.reset_callback
        )
        self.srv_set_targets = self.create_service(
            SetTargets,
            "/educational_pid/set_targets",
            self.set_targets_callback,
        )


def main(args=None):
    rclpy.init(args=args)
    node = EducationalPidNode()
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
