/**
 * @file wrench_planner.cpp
 * @brief plan force and torque from path
 * @author R.Ohnishi
 * @date 2025/07/15
 *
 * @details pathからforceとtorqeを計画
 *************************************/

#include "wrench_planner/wrench_planner.hpp"

namespace planner::wrench_planner
{

WrenchPlanner::WrenchPlanner(const rclcpp::NodeOptions & options) : Node("wrench_planner", options)
{
  std::string p_pid_controller_path = this->declare_parameter("p_pid_controller_path", "");
  std::string pid_gain_yaml = this->declare_parameter("pid_gain_yaml", "");
  std::string yaml_path = p_pid_controller_path + "/" + pid_gain_yaml;
  double publish_rate_hz = this->declare_parameter("publish_rate_hz", 20.0);
  odom_timeout_s_ = this->declare_parameter("odom_timeout_s", 1.0);

  try {
    p_pid_ctrl_ = std::make_shared<controller::P_PIDController>(yaml_path);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "%s", e.what());
    rclcpp::shutdown();
  }

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("robot_force", qos);
  pub_target_ = create_publisher<p_pid_controller_msgs::msg::Targets>("targets", qos);
  sub_ = create_subscription<planner_msgs::msg::WrenchPlan>(
    "goal_current_odom", qos,
    std::bind(&WrenchPlanner::goalCurrentOdomCallback, this, std::placeholders::_1));
  sub_odom_ = create_subscription<localization_msgs::msg::Odometry>(
    "odom", qos, std::bind(&WrenchPlanner::odomCallback, this, std::placeholders::_1));

  // Publish robot_force on a fixed-rate timer instead of directly from the
  // goal_current_odom callback, so an irregular odom arrival rate (e.g. a flaky
  // DVL) doesn't starve robot_force below mavlink_driver's command_timeout_s and
  // cause the vehicle to stop and restart. _update_wrench() coasts on the most
  // recent odom sample and bails out if it's older than odom_timeout_s_.
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz),
    std::bind(&WrenchPlanner::_update_wrench, this));
}

void WrenchPlanner::_update_wrench()
{
  if (!has_goal_received_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "Waiting for the first WrenchPlan.");
    return;
  }
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

  if (!goal_current_odom_->has_master) {
    goal_current_odom_->master.x = current_odom_->pose.position.x;
    goal_current_odom_->master.y = current_odom_->pose.position.y;
    goal_current_odom_->master.roll = current_odom_->pose.orientation.x;
    goal_current_odom_->master.yaw = current_odom_->pose.orientation.z;

    if (goal_current_odom_->z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH) {
      goal_current_odom_->master.z = current_odom_->pose.position.z_depth;
    } else if (goal_current_odom_->z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_ALTITUDE) {
      goal_current_odom_->master.z = current_odom_->pose.position.z_altitude;
    }
  }

  if (!goal_current_odom_->has_slave) {
    goal_current_odom_->slave.x = current_odom_->twist.linear.x;
    goal_current_odom_->slave.y = current_odom_->twist.linear.y;
    goal_current_odom_->slave.roll = current_odom_->twist.angular.x;
    goal_current_odom_->slave.yaw = current_odom_->twist.angular.z;

    if (goal_current_odom_->z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH) {
      goal_current_odom_->slave.z = current_odom_->twist.linear.z_depth;
    } else if (goal_current_odom_->z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_ALTITUDE) {
      goal_current_odom_->slave.z = current_odom_->twist.linear.z_altitude;
    }
  }

  const uint8_t & z_mode = goal_current_odom_->z_mode;
  const uint8_t & reset = goal_current_odom_->reset;
  const auto & target_pose = goal_current_odom_->targets;
  const auto & current_pose = goal_current_odom_->master;
  const auto & current_twst = goal_current_odom_->slave;

  auto msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();

  if (reset > 0) {
    if (reset & 0b00000001) p_pid_ctrl_->pid_x_reset();
    if (reset & 0b00000010) p_pid_ctrl_->pid_y_reset();
    if (reset & 0b00000100) p_pid_ctrl_->pid_z_reset();
    if (reset & 0b00001000) p_pid_ctrl_->pid_roll_reset();
    if (reset & 0b00010000) p_pid_ctrl_->pid_yaw_reset();
    RCLCPP_INFO(this->get_logger(), "P_PID reset: %d", reset);
  } else if (pre_z_mode_ != z_mode) {
    RCLCPP_INFO(this->get_logger(), "z-axis P_PID reset");
    pre_z_mode_ = z_mode;
    p_pid_ctrl_->pid_z_reset();
  }

  // x/y (surge/sway) are controlled entirely in body frame: rotate the world-frame
  // velocity measurement and world-frame position error into body frame *before*
  // running the PID, instead of running the PID in world frame and rotating the
  // output afterward. The latter only works if the x and y loops share identical
  // gains (rotation commutes with a scalar operator); with independently-tuned
  // gains per axis, rotating the output mixes the two loops' outputs together and
  // leaks x's force into y (or vice versa) whenever yaw != 0. Mirrors the same fix
  // applied to p_pid_controller/test_p_pid.cpp.
  double yaw_rad = current_pose.yaw * std::numbers::pi / 180;
  double body_vx = current_twst.x * cos(yaw_rad) + current_twst.y * sin(yaw_rad);
  double body_vy = -current_twst.x * sin(yaw_rad) + current_twst.y * cos(yaw_rad);

  double error_x_world = target_pose.x - current_pose.x;
  double error_y_world = target_pose.y - current_pose.y;
  double error_x_body = error_x_world * cos(yaw_rad) + error_y_world * sin(yaw_rad);
  double error_y_body = -error_x_world * sin(yaw_rad) + error_y_world * cos(yaw_rad);

  double force_x = p_pid_ctrl_->pid_x_update(body_vx, 0.0, error_x_body);
  double force_y = p_pid_ctrl_->pid_y_update(body_vy, 0.0, error_y_body);
  double force_z = p_pid_ctrl_->pid_z_update(current_twst.z, current_pose.z, target_pose.z);

  double torque_x =
    p_pid_ctrl_->pid_roll_update(current_twst.roll, current_pose.roll, target_pose.roll);

  double target_yaw = target_pose.yaw;
  if (target_pose.yaw - current_pose.yaw < -180) target_yaw += 360;
  if (target_pose.yaw - current_pose.yaw > 180) target_yaw -= 360;
  double torque_z = p_pid_ctrl_->pid_yaw_update(current_twst.yaw, current_pose.yaw, target_yaw);

  // force_x/force_y are already body-frame (surge/sway): no further rotation needed
  // before publishing, unlike the previous world-frame-then-rotate approach.

  RCLCPP_DEBUG(
    this->get_logger(), "P-PID -> x: %f  y: %f  z: %f  z_mode: %u  roll: %f  yaw: %f", force_x,
    force_y, force_z, z_mode, torque_x, torque_z);

  {
    msg->wrench.force.x = force_x;
    msg->wrench.force.y = force_y;
    msg->wrench.force.z = force_z;
    msg->wrench.torque.x = torque_x;
    msg->wrench.torque.z = torque_z;
  }
  pub_->publish(std::move(msg));

  {
    auto targets = std::make_unique<p_pid_controller_msgs::msg::Targets>();

    targets->header.stamp = this->get_clock()->now();
    targets->z_mode = z_mode;
    targets->pose = target_pose;
    pub_target_->publish(std::move(targets));
  }
}

void WrenchPlanner::goalCurrentOdomCallback(const planner_msgs::msg::WrenchPlan::SharedPtr msg)
{
  goal_current_odom_ = msg;
  has_goal_received_ = true;
  // _update_wrench() is no longer called here: it runs on a fixed-rate timer (see
  // the constructor) so robot_force keeps publishing smoothly even if goal or odom
  // messages arrive at an irregular rate, instead of being gated by each callback.
}

void WrenchPlanner::odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = msg;
  has_odom_received_ = true;
  last_odom_time_ = this->get_clock()->now();
}

}  // namespace planner::wrench_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planner::wrench_planner::WrenchPlanner)
