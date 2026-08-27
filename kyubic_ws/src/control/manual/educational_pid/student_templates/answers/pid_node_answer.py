#!/usr/bin/env python3
"""TA用解答：PIDノードのTopic、Service、状態Publish。"""

import math

import rclpy
from geometry_msgs.msg import WrenchStamped
from joy_common_msgs.msg import Joy
from localization_msgs.msg import Odometry
from p_pid_controller_msgs.msg import Targets
from p_pid_controller_msgs.srv import SetTargets
from rclpy.parameter import Parameter
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger

from educational_pid.controller_node import EducationalPidControllerBase
from educational_pid.pid_core import normalize_angle_error


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

    def set_targets_callback(self, request, response):
        target_depth = float(request.target_depth)
        target_yaw = float(request.target_yaw)
        if not math.isfinite(target_depth) or not math.isfinite(target_yaw):
            response.success = False
            response.message = "Targets must be finite numbers"
            return response
        if target_depth < 0.0:
            response.success = False
            response.message = "target_depth must be greater than or equal to 0"
            return response

        target_yaw = normalize_angle_error(target_yaw, 0.0)
        results = self.set_parameters(
            [
                Parameter("target_depth", value=target_depth),
                Parameter("target_yaw", value=target_yaw),
            ]
        )
        if not all(result.successful for result in results):
            response.success = False
            response.message = "Failed to update target parameters"
            return response

        self._reset_pid_state()
        self._update_control_errors()
        self.publish_targets()
        self.publish_pid_state()
        response.success = True
        response.message = f"Targets updated: depth={target_depth:.3f}, yaw={target_yaw:.3f}"
        return response

    def _update_control_errors(self):
        target_depth = self.get_parameter("target_depth").value
        target_yaw = self.get_parameter("target_yaw").value
        self.depth_error = float(target_depth) - self.current_depth
        self.yaw_error = normalize_angle_error(target_yaw, self.current_yaw)

    def publish_pid_state(self):
        values_and_publishers = (
            (self.depth_error, self.pub_error_depth),
            (self.yaw_error, self.pub_error_yaw),
            (self.integral_depth, self.pub_integral_depth),
            (self.integral_yaw, self.pub_integral_yaw),
        )
        for value, publisher in values_and_publishers:
            msg = Float64()
            msg.data = float(value)
            publisher.publish(msg)

    def reset_callback(self, _request, response):
        self._reset_pid_state()
        self.publish_pid_state()
        response.success = True
        response.message = "PID integral and derivative history reset"
        return response

    def _reset_pid_state(self):
        self.prev_error_depth = 0.0
        self.integral_depth = 0.0
        self.depth_pid_initialized = False
        self.prev_error_yaw = 0.0
        self.integral_yaw = 0.0
        self.yaw_pid_initialized = False
        self.last_control_time = None


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
