#!/usr/bin/env python3
"""TA用：Day 4 dry-run Safety Gate練習問題の解答。"""

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
        self.pub_safe_wrench = self.create_publisher(WrenchStamped, "safe_robot_force", 10)
        self.sub_raw_wrench = self.create_subscription(
            WrenchStamped,
            "raw_robot_force",
            self.raw_wrench_callback,
            10,
        )
        self.srv_arm = self.create_service(
            SetBool,
            "/educational_pid_day4/student_arm",
            self.arm_callback,
        )
        self.srv_stop = self.create_service(
            Trigger,
            "/educational_pid_day4/student_emergency_stop",
            self.emergency_stop_callback,
        )
        self.timer = self.create_timer(0.05, self.publish_loop)
        self.get_logger().warn("STUDENT DRY-RUN NODE: never remap this output to the actuator")

    def raw_wrench_callback(self, msg):
        values = sanitize_command(
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.z,
            allow_manual_xy=False,
            allow_depth=True,
            allow_yaw=True,
            max_force_x=0.0,
            max_force_y=0.0,
            max_force_z=2.0,
            max_torque_z=0.5,
        )
        safe_msg = WrenchStamped()
        safe_msg.header = msg.header
        safe_msg.wrench.force.x = values[0]
        safe_msg.wrench.force.y = values[1]
        safe_msg.wrench.force.z = values[2]
        safe_msg.wrench.torque.z = values[3]
        self.last_command = safe_msg

    def arm_callback(self, request, response):
        self.armed = bool(request.data)
        response.success = True
        response.message = "student gate armed" if self.armed else "student gate disarmed"
        return response

    def emergency_stop_callback(self, _request, response):
        self.armed = False
        self.publish_zero_wrench()
        response.success = True
        response.message = "student gate stopped; zero Wrench published"
        return response

    def publish_loop(self):
        if self.armed and self.last_command is not None:
            self.last_command.header.stamp = self.get_clock().now().to_msg()
            self.pub_safe_wrench.publish(self.last_command)
        else:
            self.publish_zero_wrench()

    def publish_zero_wrench(self):
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
