#!/usr/bin/env python3
"""練習問題：PIDノードのTopic、Service、状態Publishを完成させる。"""

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
        """課題1：Publisher、Subscriber、Service Serverを作る。"""
        # Publisher: robot_force, targets, integral_depth, integral_yaw,
        #            error_depth, error_yaw
        # Subscriber: joy_common, depth_odom, imu_odom
        # Service: /educational_pid/enable, /educational_pid/reset_pid,
        #          /educational_pid/set_targets
        raise NotImplementedError("課題1：ROS 2インターフェースを作成する")

    def set_targets_callback(self, request, response):
        """課題2：Service Requestから目標深度と目標Yawを更新する。"""
        # 1. Requestから目標値を取り出す。
        # 2. NaN・無限大と負の深度を拒否する。
        # 3. 目標Yawを[-180, 180)へ正規化する。
        # 4. Parameterをself.set_parameters()で更新する。
        # 5. PID状態をResetし、目標値とPID状態をPublishする。
        # 6. Responseを設定してreturnする。
        raise NotImplementedError("課題2：目標値変更Serviceを実装する")

    def _update_control_errors(self):
        """課題3-A：深度・Yaw誤差を保存する。"""
        # 深度誤差：target_depth - current_depth
        # Yaw誤差：normalize_angle_error(target_yaw, current_yaw)
        raise NotImplementedError("課題3-A：現在誤差を更新する")

    def publish_pid_state(self):
        """課題3-B：誤差と積分誤差をFloat64でPublishする。"""
        # depth_error、yaw_error、integral_depth、integral_yawを
        # それぞれ対応するPublisherから送る。
        raise NotImplementedError("課題3-B：PID状態TopicをPublishする")

    def reset_callback(self, _request, response):
        """課題4-A：PID内部状態をResetするService Callbackを作る。"""
        # _reset_pid_state()、publish_pid_state()を順に呼び、
        # Trigger Responseを成功にしてreturnする。
        raise NotImplementedError("課題4-A：Reset Serviceを実装する")

    def _reset_pid_state(self):
        """課題4-B：PIDの履歴を初期化する。"""
        raise NotImplementedError("課題4-B：PID内部状態を初期化する")


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
