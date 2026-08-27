#!/usr/bin/env python3
"""Student exercise node. This executable is for isolated dry-run only."""

from geometry_msgs.msg import WrenchStamped
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from educational_pid_robot_prep.safety_core import sanitize_command


class StudentSafetyGate(Node):
    def __init__(self):
        super().__init__("educational_pid_student_safety_gate")
        self.armed = False
        self.last_command = None

        # TODO 1: WrenchStamped Publisherを作る。
        # Topic名: safe_robot_force, QoS depth: 10
        self.pub_safe_wrench = None

        # TODO 2: WrenchStamped Subscriberを作る。
        # Topic名: raw_robot_force, callback: self.raw_wrench_callback, QoS: 10
        self.sub_raw_wrench = None

        # TODO 3: SetBool Service Serverを作る。
        # Service名: /educational_pid_day4/student_arm
        # callback: self.arm_callback
        self.srv_arm = None

        # TODO 4: Trigger Service Serverを作る。
        # Service名: /educational_pid_day4/student_emergency_stop
        # callback: self.emergency_stop_callback
        self.srv_stop = None

        self.timer = self.create_timer(0.05, self.publish_loop)
        self.get_logger().warn("STUDENT DRY-RUN NODE: never remap this output to the actuator")

    def raw_wrench_callback(self, msg):
        # TODO 5: sanitize_command()でZ推力とYawトルクを制限する。
        # X・Y手動推力は無効、Z上限2.0 N、Yaw上限0.5 N mとする。
        values = None

        if values is None:
            return
        safe_msg = WrenchStamped()
        safe_msg.header = msg.header
        safe_msg.wrench.force.x = values[0]
        safe_msg.wrench.force.y = values[1]
        safe_msg.wrench.force.z = values[2]
        safe_msg.wrench.torque.z = values[3]
        self.last_command = safe_msg

    def arm_callback(self, request, response):
        # TODO 6: request.dataをself.armedへ保存し、Responseを返す。
        response.success = False
        response.message = "TODO 6 is not complete"
        return response

    def emergency_stop_callback(self, _request, response):
        # TODO 7: armedをFalseにして、ゼロWrenchを1回Publishする。
        response.success = False
        response.message = "TODO 7 is not complete"
        return response

    def publish_loop(self):
        if self.pub_safe_wrench is None:
            return
        if self.armed and self.last_command is not None:
            self.last_command.header.stamp = self.get_clock().now().to_msg()
            self.pub_safe_wrench.publish(self.last_command)
        else:
            self.publish_zero_wrench()

    def publish_zero_wrench(self):
        if self.pub_safe_wrench is None:
            return
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_safe_wrench.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StudentSafetyGate()
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
