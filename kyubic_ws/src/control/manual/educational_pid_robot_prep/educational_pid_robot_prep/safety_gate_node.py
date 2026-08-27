#!/usr/bin/env python3
"""Double-arm safety gate between the educational PID and the actuator."""

from geometry_msgs.msg import WrenchStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from educational_pid_robot_prep.safety_core import sanitize_command


class SafetyGateNode(Node):
    """Publish a bounded Wrench only while every safety condition is valid."""

    def __init__(self):
        super().__init__("educational_pid_safety_gate")
        self._declare_parameters()

        self.armed = False
        self.last_command = None
        self.last_command_time = None
        self.last_heartbeat_time = None
        self.fault_reason = ""

        self.pub_safe_wrench = self.create_publisher(
            WrenchStamped,
            "safe_robot_force",
            10,
        )
        self.pub_armed = self.create_publisher(Bool, "armed", 10)
        self.pub_fault = self.create_publisher(String, "fault", 10)
        self.sub_raw_wrench = self.create_subscription(
            WrenchStamped,
            "raw_robot_force",
            self.raw_wrench_callback,
            10,
        )
        self.sub_heartbeat = self.create_subscription(
            Bool,
            "heartbeat",
            self.heartbeat_callback,
            10,
        )
        self.srv_arm = self.create_service(
            SetBool,
            "/educational_pid_day4/arm",
            self.arm_callback,
        )
        self.srv_emergency_stop = self.create_service(
            Trigger,
            "/educational_pid_day4/emergency_stop",
            self.emergency_stop_callback,
        )

        self.timer = self.create_timer(
            self.get_parameter("publish_period").value,
            self.publish_loop,
        )

        self.get_logger().warn("Safety gate started DISARMED; output is zero")
        raw_topic = self.resolve_topic_name("raw_robot_force")
        safe_topic = self.resolve_topic_name("safe_robot_force")
        self.get_logger().info(f"raw={raw_topic} safe={safe_topic}")

    def _declare_parameters(self):
        self.declare_parameter("allow_arm", True)
        self.declare_parameter("allow_manual_xy", False)
        self.declare_parameter("allow_depth", False)
        self.declare_parameter("allow_yaw", False)
        self.declare_parameter("max_force_x", 0.0)
        self.declare_parameter("max_force_y", 0.0)
        self.declare_parameter("max_force_z", 2.0)
        self.declare_parameter("max_torque_z", 0.5)
        self.declare_parameter("publish_period", 0.05)
        self.declare_parameter("command_timeout", 0.3)
        self.declare_parameter("require_single_input_publisher", True)
        self.declare_parameter("require_sole_output_publisher", True)
        self.declare_parameter("require_output_subscriber", False)
        self.declare_parameter("required_output_subscriber_name", "")
        self.declare_parameter("require_heartbeat", False)
        self.declare_parameter("heartbeat_timeout", 1.0)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def raw_wrench_callback(self, msg):
        try:
            values = sanitize_command(
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
                msg.wrench.torque.z,
                allow_manual_xy=self.get_parameter("allow_manual_xy").value,
                allow_depth=self.get_parameter("allow_depth").value,
                allow_yaw=self.get_parameter("allow_yaw").value,
                max_force_x=self.get_parameter("max_force_x").value,
                max_force_y=self.get_parameter("max_force_y").value,
                max_force_z=self.get_parameter("max_force_z").value,
                max_torque_z=self.get_parameter("max_torque_z").value,
            )
        except ValueError as error:
            self._disarm(str(error))
            return

        safe_msg = WrenchStamped()
        safe_msg.header = msg.header
        safe_msg.header.frame_id = "base_link"
        safe_msg.wrench.force.x = values[0]
        safe_msg.wrench.force.y = values[1]
        safe_msg.wrench.force.z = values[2]
        safe_msg.wrench.torque.z = values[3]
        self.last_command = safe_msg
        self.last_command_time = self._now_sec()

    def heartbeat_callback(self, msg):
        if msg.data:
            self.last_heartbeat_time = self._now_sec()

    def _endpoint_summary(self, endpoints):
        if not endpoints:
            return "none"
        return ", ".join(
            f"{info.node_namespace.rstrip('/')}/{info.node_name}" for info in endpoints
        )

    def _check_graph(self):
        raw_topic = self.resolve_topic_name("raw_robot_force")
        safe_topic = self.resolve_topic_name("safe_robot_force")
        raw_publishers = self.get_publishers_info_by_topic(raw_topic)
        safe_publishers = self.get_publishers_info_by_topic(safe_topic)
        safe_subscribers = self.get_subscriptions_info_by_topic(safe_topic)

        if self.get_parameter("require_single_input_publisher").value:
            if len(raw_publishers) != 1:
                return (
                    False,
                    f"raw command publisher must be exactly 1; found "
                    f"{len(raw_publishers)} ({self._endpoint_summary(raw_publishers)})",
                )

        if self.get_parameter("require_sole_output_publisher").value:
            if len(safe_publishers) != 1:
                return (
                    False,
                    f"safe output publisher must be this gate only; found "
                    f"{len(safe_publishers)} ({self._endpoint_summary(safe_publishers)})",
                )

        if self.get_parameter("require_output_subscriber").value:
            if not safe_subscribers:
                return False, "safe output has no subscriber"
            required_name = self.get_parameter("required_output_subscriber_name").value
            if required_name and not any(
                info.node_name == required_name for info in safe_subscribers
            ):
                return (
                    False,
                    f"required subscriber '{required_name}' not found; "
                    f"found {self._endpoint_summary(safe_subscribers)}",
                )

        return True, "ROS graph is valid"

    def _check_ready(self, now):
        if not self.get_parameter("allow_arm").value:
            return False, "allow_arm is false"
        if self.last_command_time is None:
            return False, "no raw Wrench has been received"
        if now - self.last_command_time > self.get_parameter("command_timeout").value:
            return False, "raw Wrench is stale"
        if self.get_parameter("require_heartbeat").value:
            if self.last_heartbeat_time is None:
                return False, "no true heartbeat has been received"
            if now - self.last_heartbeat_time > self.get_parameter("heartbeat_timeout").value:
                return False, "heartbeat is stale"
        return self._check_graph()

    def arm_callback(self, request, response):
        if not request.data:
            self._disarm("operator requested disarm")
            response.success = True
            response.message = "safety gate disarmed; zero Wrench is being published"
            return response

        ready, reason = self._check_ready(self._now_sec())
        if not ready:
            self._disarm(reason)
            response.success = False
            response.message = f"not armed: {reason}"
            return response

        self.armed = True
        self.fault_reason = ""
        response.success = True
        response.message = "safety gate ARMED"
        self.get_logger().warn("SAFETY GATE ARMED")
        return response

    def emergency_stop_callback(self, _request, response):
        self._disarm("emergency stop service called")
        self.publish_zero_wrench()
        response.success = True
        response.message = "emergency stop: gate disarmed and zero Wrench published"
        return response

    def _disarm(self, reason):
        was_armed = self.armed
        self.armed = False
        self.fault_reason = reason
        if was_armed:
            self.get_logger().error(f"Safety gate DISARMED: {reason}")

    def publish_loop(self):
        if self.armed:
            ready, reason = self._check_ready(self._now_sec())
            if not ready:
                self._disarm(reason)

        if self.armed and self.last_command is not None:
            self.last_command.header.stamp = self.get_clock().now().to_msg()
            self.pub_safe_wrench.publish(self.last_command)
        else:
            self.publish_zero_wrench()

        armed_msg = Bool()
        armed_msg.data = self.armed
        self.pub_armed.publish(armed_msg)
        fault_msg = String()
        fault_msg.data = self.fault_reason
        self.pub_fault.publish(fault_msg)

    def publish_zero_wrench(self):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.pub_safe_wrench.publish(msg)

    def destroy_node(self):
        self.armed = False
        self.publish_zero_wrench()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyGateNode()
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
