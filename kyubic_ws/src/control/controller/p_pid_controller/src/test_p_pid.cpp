/**
 * @file test_p_pid.cpp
 * @brief Test code for P-PID Controller
 * @author R.Ohnishi
 * @date 2025/05/14
 *
 * @details P-PID制御のテストコード
 **************************************************/

#include "p_pid_controller/test_p_pid.hpp"

using namespace std::chrono_literals;

namespace controller
{

TestPPID::TestPPID(const rclcpp::NodeOptions & options) : Node("test_p_pid", options)
{
  std::string yaml_path = this->declare_parameter("pid_gain_yaml", "");
  double publish_rate_hz = this->declare_parameter("publish_rate_hz", 20.0);
  odom_timeout_s_ = this->declare_parameter("odom_timeout_s", 1.0);

  try {
    p_pid_ctrl_ = std::make_shared<controller::P_PIDController>(yaml_path);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "%s", e.what());
    rclcpp::shutdown();
  }

  // Create messages instance
  odom_ = std::make_shared<localization_msgs::msg::Odometry>();
  targets_ = std::make_shared<p_pid_controller_msgs::msg::Targets>();

  // ROS 2 communication
  rclcpp::QoS qos(rclcpp::KeepLast(1));

  pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("robot_force", qos);
  sub_targets_ = create_subscription<p_pid_controller_msgs::msg::Targets>(
    "targets", qos, std::bind(&TestPPID::callback_target, this, std::placeholders::_1));
  sub_odom_ = create_subscription<localization_msgs::msg::Odometry>(
    "odom", qos, std::bind(&TestPPID::callback_odom, this, std::placeholders::_1));

  // Publish robot_force on a fixed-rate timer instead of directly from the odom
  // callback, so an irregular odom arrival rate (e.g. a flaky DVL) doesn't starve
  // robot_force below mavlink_driver's command_timeout_s and cause the vehicle to
  // stop and restart. update() coasts on the most recent odom sample and bails out
  // if it's older than odom_timeout_s_ (see update()).
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz), std::bind(&TestPPID::update, this));
}

void TestPPID::update()
{
  if (!has_odom_received_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "Waiting for the first odometry sample.");
    return;
  }
  double odom_age_s = (this->get_clock()->now() - last_odom_time_).seconds();
  if (odom_age_s > odom_timeout_s_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Odometry is stale (%.2f s > %.2f s); withholding robot_force.", odom_age_s, odom_timeout_s_);
    return;
  }

  const auto & odom_status = odom_->status;
  const bool odom_invalid = odom_status.depth.id == common_msgs::msg::Status::ERROR ||
                            odom_status.imu.id == common_msgs::msg::Status::ERROR ||
                            odom_status.dvl.id == common_msgs::msg::Status::ERROR;

  if (!has_target_received_) {
    if (odom_invalid) {
      // Odometry has not converged yet (sensors not ready). Do not seed a
      // hold target from garbage/uninitialized pose data; wait for the first
      // valid odometry sample instead of actuating on it.
      RCLCPP_WARN(
        this->get_logger(),
        "No targets received yet and odometry is not valid yet. Waiting before actuating.");
      return;
    }
    // No operator target has arrived yet: seed a hold-current-position target
    // from the first valid odometry sample instead of driving toward the
    // default-constructed Targets (x=y=z=roll=yaw=0), which would otherwise
    // command full-saturation thrust toward the world origin.
    targets_->z_mode = p_pid_controller_msgs::msg::Targets::Z_MODE_DEPTH;
    targets_->pose.x = odom_->pose.position.x;
    targets_->pose.y = odom_->pose.position.y;
    targets_->pose.z = odom_->pose.position.z_depth;
    targets_->pose.roll = odom_->pose.orientation.x;
    targets_->pose.yaw = odom_->pose.orientation.z;
    has_target_received_ = true;
    RCLCPP_WARN(
      this->get_logger(),
      "No targets received yet. Holding current position: x=%f y=%f z=%f roll=%f yaw=%f",
      targets_->pose.x, targets_->pose.y, targets_->pose.z, targets_->pose.roll,
      targets_->pose.yaw);
  }

  const uint8_t & z_mode = targets_->z_mode;
  const auto & target_pose = targets_->pose;
  const auto & current_pose = odom_->pose;
  const auto & current_twst = odom_->twist;

  auto msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();

  if (odom_invalid) {
    RCLCPP_ERROR(this->get_logger(), "The current odometry is invalid");

    double force_z = 0.0;
    if (
      odom_status.depth.id != common_msgs::msg::Status::ERROR &&
      z_mode == p_pid_controller_msgs::msg::Targets::Z_MODE_DEPTH) {
      force_z = p_pid_ctrl_->pid_z_update(
        current_twst.linear.z_depth, current_pose.position.z_depth, target_pose.z);
    } else if (
      odom_status.dvl.id != common_msgs::msg::Status::ERROR &&
      z_mode == p_pid_controller_msgs::msg::Targets::Z_MODE_ALTITUDE) {
      force_z = -p_pid_ctrl_->pid_z_update(
        current_twst.linear.z_altitude, current_pose.position.z_altitude, target_pose.z);
    }

    {
      msg->wrench.force.z = force_z;
      RCLCPP_WARN(this->get_logger(), "only z-axis control: %lf[N]", force_z);
    }
  } else {
    const auto & target_twist = targets_->twist;
    const uint8_t & velocity_axis_mask = targets_->velocity_axis_mask;

    const bool x_velocity_mode =
      velocity_axis_mask & p_pid_controller_msgs::msg::Targets::VELOCITY_AXIS_X;
    const bool y_velocity_mode =
      velocity_axis_mask & p_pid_controller_msgs::msg::Targets::VELOCITY_AXIS_Y;
    const bool z_velocity_mode =
      velocity_axis_mask & p_pid_controller_msgs::msg::Targets::VELOCITY_AXIS_Z;
    const bool roll_velocity_mode =
      velocity_axis_mask & p_pid_controller_msgs::msg::Targets::VELOCITY_AXIS_ROLL;
    const bool yaw_velocity_mode =
      velocity_axis_mask & p_pid_controller_msgs::msg::Targets::VELOCITY_AXIS_YAW;

    // x/y (surge/sway) are controlled entirely in body frame: rotate the world-frame
    // velocity measurement and world-frame position error into body frame *before*
    // running the PID, instead of running the PID in world frame and rotating the
    // output afterward. The latter only works if the x and y loops share identical
    // gains (rotation commutes with a scalar operator); with independently-tuned
    // gains per axis (e.g. x tuned, y still at zero), rotating the output mixes the
    // two loops' outputs together and leaks x's force into y whenever yaw != 0.
    double yaw_rad = current_pose.orientation.z * std::numbers::pi / 180;
    double body_vx = current_twst.linear.x * cos(yaw_rad) + current_twst.linear.y * sin(yaw_rad);
    double body_vy = -current_twst.linear.x * sin(yaw_rad) + current_twst.linear.y * cos(yaw_rad);

    double error_x_world = target_pose.x - current_pose.position.x;
    double error_y_world = target_pose.y - current_pose.position.y;
    double error_x_body = error_x_world * cos(yaw_rad) + error_y_world * sin(yaw_rad);
    double error_y_body = -error_x_world * sin(yaw_rad) + error_y_world * cos(yaw_rad);

    double force_x = x_velocity_mode ? p_pid_ctrl_->pid_x_update_velocity(body_vx, target_twist.x)
                                     : p_pid_ctrl_->pid_x_update(body_vx, 0.0, error_x_body);
    double force_y = y_velocity_mode ? p_pid_ctrl_->pid_y_update_velocity(body_vy, target_twist.y)
                                     : p_pid_ctrl_->pid_y_update(body_vy, 0.0, error_y_body);

    double force_z = 0.0;
    if (pre_z_mode != z_mode) {
      RCLCPP_INFO(this->get_logger(), "z-axis P_PID reset");
      pre_z_mode = z_mode;
      p_pid_ctrl_->pid_z_reset();
    }
    if (z_mode == p_pid_controller_msgs::msg::Targets::Z_MODE_DEPTH) {
      force_z = z_velocity_mode
                  ? p_pid_ctrl_->pid_z_update_velocity(current_twst.linear.z_depth, target_twist.z)
                  : p_pid_ctrl_->pid_z_update(
                      current_twst.linear.z_depth, current_pose.position.z_depth, target_pose.z);
    } else if (z_mode == p_pid_controller_msgs::msg::Targets::Z_MODE_ALTITUDE) {
      force_z =
        z_velocity_mode
          ? -p_pid_ctrl_->pid_z_update_velocity(current_twst.linear.z_altitude, target_twist.z)
          : -p_pid_ctrl_->pid_z_update(
              current_twst.linear.z_altitude, current_pose.position.z_altitude, target_pose.z);
    } else {
      RCLCPP_ERROR(this->get_logger(), "z_mode is failure");
      return;
    }

    p_pid_ctrl_->set_roll_offset(current_pose.orientation.x);
    double torque_x =
      roll_velocity_mode
        ? p_pid_ctrl_->pid_roll_update_velocity(current_twst.angular.x, target_twist.roll)
        : p_pid_ctrl_->pid_roll_update(
            current_twst.angular.x, current_pose.orientation.x, target_pose.roll);

    double torque_z;
    if (yaw_velocity_mode) {
      torque_z = p_pid_ctrl_->pid_yaw_update_velocity(current_twst.angular.z, target_twist.yaw);
    } else {
      double target_yaw = target_pose.yaw;
      if (target_pose.yaw - current_pose.orientation.z < -180) target_yaw += 360;
      if (target_pose.yaw - current_pose.orientation.z > 180) target_yaw -= 360;
      torque_z =
        p_pid_ctrl_->pid_yaw_update(current_twst.angular.z, current_pose.orientation.z, target_yaw);
    }

    // force_x/force_y are already body-frame (surge/sway): no further rotation needed
    // before publishing, unlike the previous world-frame-then-rotate approach.

    RCLCPP_INFO(
      this->get_logger(), "P-PID -> x: %f  y: %f  z: %f  z_mode: %u  roll: %f  yaw: %f", force_x,
      force_y, force_z, z_mode, torque_x, torque_z);

    {
      msg->wrench.force.x = force_x;
      msg->wrench.force.y = force_y;
      msg->wrench.force.z = force_z;
      msg->wrench.torque.x = torque_x;
      msg->wrench.torque.z = torque_z;
    }
  }
  msg->header.stamp = this->get_clock()->now();
  pub_->publish(std::move(msg));
}

void TestPPID::callback_target(p_pid_controller_msgs::msg::Targets::UniquePtr msg)
{
  if (msg->reset > 0) {
    if (msg->reset & p_pid_controller_msgs::msg::Targets::RESET_X) p_pid_ctrl_->pid_x_reset();
    if (msg->reset & p_pid_controller_msgs::msg::Targets::RESET_Y) p_pid_ctrl_->pid_y_reset();
    if (msg->reset & p_pid_controller_msgs::msg::Targets::RESET_Z) p_pid_ctrl_->pid_z_reset();
    if (msg->reset & p_pid_controller_msgs::msg::Targets::RESET_ROLL) p_pid_ctrl_->pid_roll_reset();
    if (msg->reset & p_pid_controller_msgs::msg::Targets::RESET_YAW) p_pid_ctrl_->pid_yaw_reset();
    RCLCPP_INFO(this->get_logger(), "P_PID reset: %d", msg->reset);
  }
  targets_ = std::move(msg);
  has_target_received_ = true;
}

void TestPPID::callback_odom(localization_msgs::msg::Odometry::UniquePtr msg)
{
  odom_ = std::move(msg);
  has_odom_received_ = true;
  last_odom_time_ = this->get_clock()->now();
  // update() is no longer called here: it runs on a fixed-rate timer (see the
  // constructor) so robot_force keeps publishing smoothly even if odom arrives at
  // an irregular rate, instead of being gated by every single odom callback.
}

}  // namespace controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(controller::TestPPID)
