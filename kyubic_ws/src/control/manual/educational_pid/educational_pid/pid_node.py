#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from joy_common_msgs.msg import Joy
from localization_msgs.msg import Odometry


# ==============================================================
# インターン生向け: 以下のPID制御関数を完成させよう！
# (現在は解答がすべて書かれています。後でコメントアウト・消去してください)
# ==============================================================
def calculate_pid(target, current, kp, ki, kd, prev_error, integral, dt):
    """
    PID制御の計算を行う関数
    :param target: 目標値
    :param current: 現在値
    :param kp: 比例ゲイン (P制御)
    :param ki: 積分ゲイン (I制御)
    :param kd: 微分ゲイン (D制御)
    :param prev_error: 前回の誤差
    :param integral: これまでの誤差の蓄積(積分値)
    :param dt: 経過時間(秒)
    :return: 計算された操作量(推力), 今回の誤差, 新しい積分値
    """
    # 1. 誤差(error)の計算: 目標値から現在値を引く
    error = target - current

    # 2. P項(比例)の計算: ゲインと誤差を掛ける
    p_term = kp * error

    # 3. I項(積分)の計算: 誤差を蓄積(足し合わせ)してゲインを掛ける
    integral = integral + (error * dt)
    i_term = ki * integral

    # 4. D項(微分)の計算: 今回の誤差と前回の誤差の変化量を求めてゲインを掛ける
    derivative = (error - prev_error) / dt if dt > 0.0 else 0.0
    d_term = kd * derivative

    # 5. 合計の操作量を出力
    output = p_term + i_term + d_term

    return output, error, integral


# ==============================================================


class EducationalPidNode(Node):
    def __init__(self):
        super().__init__("educational_pid_node")

        # パラメータ (ゲインの調整はここで行う)
        self.declare_parameter("kp_depth", 1.0)
        self.declare_parameter("ki_depth", 0.0)
        self.declare_parameter("kd_depth", 0.1)
        self.declare_parameter("kp_yaw", 1.0)
        self.declare_parameter("ki_yaw", 0.0)
        self.declare_parameter("kd_yaw", 0.1)

        # ROS 2 通信の設定
        self.pub_wrench = self.create_publisher(WrenchStamped, "robot_force", 10)
        self.sub_joy = self.create_subscription(Joy, "joy_common", self.joy_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        # 状態変数
        self.current_depth = 0.0
        self.current_yaw = 0.0

        # =======================================================
        # インターン生向け: 以下の目標値（Target）を書き換えてみよう！
        # =======================================================
        self.target_depth = 1.0  # 目標水深 (例: 1.0 = 水深1m)
        self.target_yaw = 0.0  # 目標方位 (例: 90.0 = 90度回転)
        # =======================================================

        # PID用変数
        self.prev_error_depth = 0.0
        self.integral_depth = 0.0
        self.prev_error_yaw = 0.0
        self.integral_yaw = 0.0

        # ボタンの長押し防止用フラグ
        self.prev_up = False
        self.prev_down = False
        self.prev_left = False
        self.prev_right = False

        # タイマー（制御周期：0.1秒 = 10Hz）
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # マニュアル操作用のXY出力の一時保存
        self.manual_force_x = 0.0
        self.manual_force_y = 0.0

        self.get_logger().info("Educational PID Node is starting!")
        self.get_logger().info("Edit target_depth and target_yaw in the code to change targets.")

    def odom_callback(self, msg):
        """オドメトリ(現在位置・姿勢)を受信したときの処理"""
        # 水中ロボットの深度(z_depth)とヨー角(orientation.z)を取得
        self.current_depth = msg.pose.position.z_depth
        self.current_yaw = msg.pose.orientation.z

    def joy_callback(self, msg):
        """ジョイスティック(コントローラー)の入力を受信したときの処理"""
        # 左スティックの上下左右でX/Yの推力を決定 (既存のjoy2wrenchと同じ仕様)
        self.manual_force_x = msg.stick.ly
        self.manual_force_y = -msg.stick.lx

        # 注意: 方位(rx)や深度(ry)のスティック入力はここで無視されます。
        # したがって、ジョイスティックで方位・深度を操作しようとしても何も起こらず、
        # プログラムで設定した target_depth と target_yaw を維持し続けます。

    def control_loop(self):
        """一定周期(10Hz)で実行される制御ループ"""

        # 現在のPIDゲインを取得
        kp_d = self.get_parameter("kp_depth").value
        ki_d = self.get_parameter("ki_depth").value
        kd_d = self.get_parameter("kd_depth").value

        kp_y = self.get_parameter("kp_yaw").value
        ki_y = self.get_parameter("ki_yaw").value
        kd_y = self.get_parameter("kd_yaw").value

        # =======================================================
        # 深度(Depth)のPID制御を実行
        # =======================================================
        force_z, self.prev_error_depth, self.integral_depth = calculate_pid(
            target=self.target_depth,
            current=self.current_depth,
            kp=kp_d,
            ki=ki_d,
            kd=kd_d,
            prev_error=self.prev_error_depth,
            integral=self.integral_depth,
            dt=self.timer_period,
        )

        # =======================================================
        # ヨー角(Yaw)のPID制御を実行
        # =======================================================
        torque_z, self.prev_error_yaw, self.integral_yaw = calculate_pid(
            target=self.target_yaw,
            current=self.current_yaw,
            kp=kp_y,
            ki=ki_y,
            kd=kd_y,
            prev_error=self.prev_error_yaw,
            integral=self.integral_yaw,
            dt=self.timer_period,
        )

        # =======================================================
        # ロボットへ推力指令(WrenchStamped)を送信
        # =======================================================
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = "base_link"

        # マニュアル操作のXY推力
        wrench_msg.wrench.force.x = self.manual_force_x
        wrench_msg.wrench.force.y = self.manual_force_y

        # PID制御で計算したZ推力とYawトルク
        wrench_msg.wrench.force.z = force_z
        wrench_msg.wrench.torque.z = torque_z

        # X/Yのトルクはゼロとする
        wrench_msg.wrench.torque.x = 0.0
        wrench_msg.wrench.torque.y = 0.0

        self.pub_wrench.publish(wrench_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EducationalPidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
