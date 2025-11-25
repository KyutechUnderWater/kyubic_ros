/**
 * @file wrench_planner.cpp
 * @brief plan force and torque from path
 * @author R.Ohnishi
 * @date 2025/07/15
 *
 * @details pathからforceとtorqeを計画
 *************************************/

#include "wrench_planner/wrench_planner.hpp"

namespace planner
{

WrenchPlanner::WrenchPlanner(const rclcpp::NodeOptions & options) : Node("wrench_planner", options)
{
  std::string p_pid_controller_path = this->declare_parameter("p_pid_controller_path", "");
  std::string pid_gain_yaml = this->declare_parameter("pid_gain_yaml", "");
  std::string yaml_path = p_pid_controller_path + "/" + pid_gain_yaml;

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
}

void WrenchPlanner::_update_wrench()
{
  const uint8_t & z_mode = goal_current_odom_->z_mode;
  const auto & target_pose = goal_current_odom_->targets;
  const auto & current_pose = goal_current_odom_->master;
  const auto & current_twst = goal_current_odom_->slave;

  auto msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();

  if (pre_z_mode != z_mode) {
    RCLCPP_INFO(this->get_logger(), "z-axis P_PID reset");
    pre_z_mode = z_mode;
    p_pid_ctrl_->pid_z_reset();
  }

  double force_x = p_pid_ctrl_->pid_x_update(current_twst.x, current_pose.x, target_pose.x);
  double force_y = p_pid_ctrl_->pid_y_update(current_twst.y, current_pose.y, target_pose.y);
  double force_z = p_pid_ctrl_->pid_z_update(current_twst.z, current_pose.z, target_pose.z);

  // double torque_x =
  //   p_pid_ctrl_->pid_roll_update(current_twst.roll, current_pose.roll, target_pose.roll);
  // rollの制御を無効化
  double torque_x = 0.0;

  double target_yaw = target_pose.yaw;
  if (target_pose.yaw - current_pose.yaw < -180) target_yaw += 360;
  if (target_pose.yaw - current_pose.yaw > 180) target_yaw -= 360;
  double torque_z = p_pid_ctrl_->pid_yaw_update(current_twst.yaw, current_pose.yaw, target_yaw);

  // z-axis transform
  double z_rad = -current_pose.yaw * std::numbers::pi / 180;
  double _force_x = force_x;
  double _force_y = force_y;
  force_x = _force_x * cos(z_rad) - _force_y * sin(z_rad);
  force_y = _force_x * sin(z_rad) + _force_y * cos(z_rad);

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
  _update_wrench();
}

}  // namespace planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planner::WrenchPlanner)
