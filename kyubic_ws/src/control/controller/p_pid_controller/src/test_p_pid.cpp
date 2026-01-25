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
}

void TestPPID::update()
{
  const uint8_t & z_mode = targets_->z_mode;
  const auto & target_pose = targets_->pose;
  const auto & odom_status = odom_->status;
  const auto & current_pose = odom_->pose;
  const auto & current_twst = odom_->twist;

  auto msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();

  if (
    odom_status.depth.id == common_msgs::msg::Status::ERROR ||
    odom_status.imu.id == common_msgs::msg::Status::ERROR ||
    odom_status.dvl.id == common_msgs::msg::Status::ERROR) {
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
    double force_x =
      p_pid_ctrl_->pid_x_update(current_twst.linear.x, current_pose.position.x, target_pose.x);
    double force_y =
      p_pid_ctrl_->pid_y_update(current_twst.linear.y, current_pose.position.y, target_pose.y);

    double force_z = 0.0;
    if (pre_z_mode != z_mode) {
      RCLCPP_INFO(this->get_logger(), "z-axis P_PID reset");
      pre_z_mode = z_mode;
      p_pid_ctrl_->pid_z_reset();
    }
    if (z_mode == p_pid_controller_msgs::msg::Targets::Z_MODE_DEPTH) {
      force_z = p_pid_ctrl_->pid_z_update(
        current_twst.linear.z_depth, current_pose.position.z_depth, target_pose.z);
    } else if (z_mode == p_pid_controller_msgs::msg::Targets::Z_MODE_ALTITUDE) {
      force_z = -p_pid_ctrl_->pid_z_update(
        current_twst.linear.z_altitude, current_pose.position.z_altitude, target_pose.z);
    } else {
      RCLCPP_ERROR(this->get_logger(), "z_mode is failure");
      return;
    }

    p_pid_ctrl_->set_roll_offset(current_pose.orientation.x);
    double torque_x = p_pid_ctrl_->pid_roll_update(
      current_twst.angular.x, current_pose.orientation.x, target_pose.roll);

    double target_yaw = target_pose.yaw;
    if (target_pose.yaw - current_pose.orientation.z < -180) target_yaw += 360;
    if (target_pose.yaw - current_pose.orientation.z > 180) target_yaw -= 360;
    double torque_z =
      p_pid_ctrl_->pid_yaw_update(current_twst.angular.z, current_pose.orientation.z, target_yaw);

    // z-axis transform
    double z_rad = -current_pose.orientation.z * std::numbers::pi / 180;
    double _force_x = force_x;
    double _force_y = force_y;
    force_x = _force_x * cos(z_rad) - _force_y * sin(z_rad);
    force_y = _force_x * sin(z_rad) + _force_y * cos(z_rad);

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
  targets_ = std::move(msg);
}

void TestPPID::callback_odom(localization_msgs::msg::Odometry::UniquePtr msg)
{
  odom_ = std::move(msg);
  update();
}

}  // namespace controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(controller::TestPPID)
