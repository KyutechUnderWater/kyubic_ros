"""Safety wrapper and control loop for the educational PID node."""

import math

from common_msgs.msg import Status
from geometry_msgs.msg import WrenchStamped
from p_pid_controller_msgs.msg import Targets
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64

from educational_pid.pid_core import calculate_pid, clamp, normalize_angle_error


class EducationalPidControllerBase(Node):
    """Base node containing control and safety logic.

    ``create_interfaces`` is implemented in ``pid_node.py`` so students can
    practise ROS 2 Topic and Service creation without editing safety logic.
    """

    def __init__(self):
        super().__init__("educational_pid_node")
        self._declare_parameters()

        self.current_depth = 0.0
        self.current_yaw = 0.0
        self.depth_valid = False
        self.yaw_valid = False
        self.last_depth_time = None
        self.last_yaw_time = None

        self.manual_force_x = 0.0
        self.manual_force_y = 0.0
        self.last_joy_time = None

        self.enabled = False
        self.prev_error_depth = 0.0
        self.integral_depth = 0.0
        self.depth_error = 0.0
        self.depth_pid_initialized = False
        self.prev_error_yaw = 0.0
        self.integral_yaw = 0.0
        self.yaw_error = 0.0
        self.yaw_pid_initialized = False
        self.last_control_time = None
        self.last_sensor_warning_time = -1.0

        self.pub_wrench = None
        self.pub_targets = None
        self.pub_integral_depth = None
        self.pub_integral_yaw = None
        self.pub_error_depth = None
        self.pub_error_yaw = None
        self.sub_joy = None
        self.sub_depth = None
        self.sub_imu = None
        self.srv_enable = None
        self.srv_reset = None
        self.srv_set_targets = None
        self.create_interfaces()

        self.timer = self.create_timer(
            self.get_parameter("control_period").value,
            self.control_loop,
        )

        self.get_logger().info("Educational PID node started in DISABLED state")
        self.get_logger().info(
            "Enable with /educational_pid/enable after checking targets and limits"
        )

    def _declare_parameters(self):
        self.declare_parameter("target_depth", 0.0)
        self.declare_parameter("target_yaw", 0.0)
        self.declare_parameter("kp_depth", 1.0)
        self.declare_parameter("ki_depth", 0.0)
        self.declare_parameter("kd_depth", 0.1)
        self.declare_parameter("kp_yaw", 0.05)
        self.declare_parameter("ki_yaw", 0.0)
        self.declare_parameter("kd_yaw", 0.01)
        self.declare_parameter("integral_limit_depth", 2.0)
        self.declare_parameter("integral_limit_yaw", 30.0)
        self.declare_parameter("max_force_z", 5.0)
        self.declare_parameter("max_torque_z", 2.0)
        self.declare_parameter("depth_feedforward", 0.0)
        self.declare_parameter("manual_force_x_scale", 52.0)
        self.declare_parameter("manual_force_y_scale", 30.0)
        self.declare_parameter("control_period", 0.1)
        self.declare_parameter("sensor_timeout", 0.5)
        self.declare_parameter("joy_timeout", 0.5)

    def create_interfaces(self):
        """Create publishers, subscriptions and services in the subclass."""
        raise NotImplementedError

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def depth_callback(self, msg):
        value = msg.pose.position.z_depth
        self.current_depth = value
        self.depth_valid = msg.status.depth.id != Status.ERROR and math.isfinite(value)
        self.last_depth_time = self._now_sec()

    def imu_callback(self, msg):
        value = msg.pose.orientation.z
        self.current_yaw = value
        self.yaw_valid = msg.status.imu.id != Status.ERROR and math.isfinite(value)
        self.last_yaw_time = self._now_sec()

    def joy_callback(self, msg):
        scale_x = self.get_parameter("manual_force_x_scale").value
        scale_y = self.get_parameter("manual_force_y_scale").value
        self.manual_force_x = msg.stick.ly * scale_x
        self.manual_force_y = -msg.stick.lx * scale_y
        self.last_joy_time = self._now_sec()

    def enable_callback(self, request, response):
        if request.data:
            now = self._now_sec()
            if not self._sensors_ready(now):
                response.success = False
                response.message = "PID not enabled: depth/IMU data are invalid or stale"
                self.enabled = False
                self.publish_zero_wrench()
                return response

            self._reset_pid_state()
            self.enabled = True
            response.success = True
            response.message = "PID enabled"
            self.get_logger().warn("PID ENABLED")
            return response

        self.enabled = False
        self._reset_pid_state()
        self.publish_zero_wrench()
        response.success = True
        response.message = "PID disabled and zero wrench published"
        self.get_logger().info("PID disabled")
        return response

    def reset_callback(self, _request, response):
        self._reset_pid_state()
        self.publish_pid_state()
        response.success = True
        response.message = "PID integral and derivative history reset"
        return response

    def set_targets_callback(self, request, response):
        """Change depth and yaw targets through an explicit ROS 2 Service."""
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

        # Avoid a derivative kick and carrying the old integral into a new goal.
        self._reset_pid_state()
        self._update_control_errors()
        self.publish_targets()
        self.publish_pid_state()
        response.success = True
        response.message = f"Targets updated: depth={target_depth:.3f}, yaw={target_yaw:.3f}"
        return response

    def _reset_pid_state(self):
        self.prev_error_depth = 0.0
        self.integral_depth = 0.0
        self.depth_pid_initialized = False
        self.prev_error_yaw = 0.0
        self.integral_yaw = 0.0
        self.yaw_pid_initialized = False
        self.last_control_time = None

    def _sensors_ready(self, now):
        if self.last_depth_time is None or self.last_yaw_time is None:
            return False
        timeout = self.get_parameter("sensor_timeout").value
        return (
            self.depth_valid
            and self.yaw_valid
            and now - self.last_depth_time <= timeout
            and now - self.last_yaw_time <= timeout
        )

    def _manual_xy(self, now):
        if self.last_joy_time is None:
            return 0.0, 0.0
        if now - self.last_joy_time > self.get_parameter("joy_timeout").value:
            return 0.0, 0.0
        return self.manual_force_x, self.manual_force_y

    def _update_control_errors(self):
        """Update observable errors even while PID output is disabled."""
        target_depth = self.get_parameter("target_depth").value
        target_yaw = self.get_parameter("target_yaw").value
        self.depth_error = float(target_depth) - self.current_depth
        self.yaw_error = normalize_angle_error(target_yaw, self.current_yaw)

    def control_loop(self):
        now = self._now_sec()
        self.publish_targets()
        self._update_control_errors()
        if not self.enabled:
            self.publish_pid_state()
            self.publish_zero_wrench()
            return

        if not self._sensors_ready(now):
            if now - self.last_sensor_warning_time >= 1.0:
                self.get_logger().error("Sensor data invalid/stale; disabling PID")
                self.last_sensor_warning_time = now
            self.enabled = False
            self._reset_pid_state()
            self.publish_pid_state()
            self.publish_zero_wrench()
            return

        configured_period = self.get_parameter("control_period").value
        dt = configured_period if self.last_control_time is None else now - self.last_control_time
        self.last_control_time = now
        if dt <= 0.0 or dt > 5.0 * configured_period:
            dt = configured_period

        force_z, self.prev_error_depth, self.integral_depth = calculate_pid(
            error=self.depth_error,
            kp=self.get_parameter("kp_depth").value,
            ki=self.get_parameter("ki_depth").value,
            kd=self.get_parameter("kd_depth").value,
            prev_error=self.prev_error_depth,
            integral=self.integral_depth,
            dt=dt,
            integral_limit=self.get_parameter("integral_limit_depth").value,
            output_limit=self.get_parameter("max_force_z").value,
            has_previous_error=self.depth_pid_initialized,
        )
        self.depth_pid_initialized = True

        yaw_prev_for_derivative = self.prev_error_yaw
        if self.yaw_pid_initialized:
            yaw_error_change = normalize_angle_error(
                self.yaw_error,
                self.prev_error_yaw,
            )
            yaw_prev_for_derivative = self.yaw_error - yaw_error_change

        torque_z, self.prev_error_yaw, self.integral_yaw = calculate_pid(
            error=self.yaw_error,
            kp=self.get_parameter("kp_yaw").value,
            ki=self.get_parameter("ki_yaw").value,
            kd=self.get_parameter("kd_yaw").value,
            prev_error=yaw_prev_for_derivative,
            integral=self.integral_yaw,
            dt=dt,
            integral_limit=self.get_parameter("integral_limit_yaw").value,
            output_limit=self.get_parameter("max_torque_z").value,
            has_previous_error=self.yaw_pid_initialized,
        )
        self.yaw_pid_initialized = True

        force_z = clamp(
            force_z + self.get_parameter("depth_feedforward").value,
            self.get_parameter("max_force_z").value,
        )
        force_x, force_y = self._manual_xy(now)
        self.publish_pid_state()
        self.publish_wrench(force_x, force_y, force_z, torque_z)

    def publish_targets(self):
        """Publish the current depth/yaw targets for rt_pose_plotter."""
        if self.pub_targets is None:
            return
        msg = Targets()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.z_mode = Targets.Z_MODE_DEPTH
        msg.pose.z = float(self.get_parameter("target_depth").value)
        msg.pose.yaw = float(self.get_parameter("target_yaw").value)
        self.pub_targets.publish(msg)

    def publish_pid_state(self):
        """Publish PID errors and integrals for observation and reset checks."""
        if self.pub_error_depth is not None:
            depth_error_msg = Float64()
            depth_error_msg.data = float(self.depth_error)
            self.pub_error_depth.publish(depth_error_msg)
        if self.pub_error_yaw is not None:
            yaw_error_msg = Float64()
            yaw_error_msg.data = float(self.yaw_error)
            self.pub_error_yaw.publish(yaw_error_msg)
        if self.pub_integral_depth is not None:
            depth_msg = Float64()
            depth_msg.data = float(self.integral_depth)
            self.pub_integral_depth.publish(depth_msg)
        if self.pub_integral_yaw is not None:
            yaw_msg = Float64()
            yaw_msg.data = float(self.integral_yaw)
            self.pub_integral_yaw.publish(yaw_msg)

    def publish_wrench(self, force_x, force_y, force_z, torque_z):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.wrench.force.x = float(force_x)
        msg.wrench.force.y = float(force_y)
        msg.wrench.force.z = float(force_z)
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = float(torque_z)
        self.pub_wrench.publish(msg)

    def publish_zero_wrench(self):
        if self.pub_wrench is not None:
            self.publish_wrench(0.0, 0.0, 0.0, 0.0)

    def destroy_node(self):
        self.enabled = False
        self.publish_zero_wrench()
        super().destroy_node()
