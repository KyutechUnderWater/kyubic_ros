/**
 * @file velocity_wrench_planner.hpp
 * @brief plan force and torque from path with velocity
 * @author R.Ohnishi
 * @date 2025/11/24
 *
 * @details pathからforceとtorqeを計画(速度制御)
 **********************************************/

#include "wrench_planner/velocity_wrench_planner.hpp"

namespace planner
{

VelocityWrenchPlanner::VelocityWrenchPlanner(const rclcpp::NodeOptions & options)
: Node("velocity_wrench_planner", options)
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  pub_ = create_publisher<planner_msgs::msg::WrenchPlan>("goal_current_odom", qos);

  plan_sub_ = create_subscription<planner_msgs::msg::WrenchPlan>(
    "wrench_plan", qos,
    std::bind(&VelocityWrenchPlanner::wrenchPlanCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<localization_msgs::msg::Odometry>(
    "odom", qos, std::bind(&VelocityWrenchPlanner::odomCallback, this, std::placeholders::_1));
}

void VelocityWrenchPlanner::wrenchPlanCallback(planner_msgs::msg::WrenchPlan::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!odom_ || !is_update) {
    RCLCPP_WARN(this->get_logger(), "Waiting for Odometry data... cannot publish WrenchPlan yet.");
    return;
  }

  _msg->slave.x = odom_->twist.linear.x;
  _msg->slave.y = odom_->twist.linear.y;
  _msg->slave.roll = odom_->twist.angular.x;
  _msg->slave.yaw = odom_->twist.angular.z;

  if (_msg->z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH) {
    _msg->slave.z = odom_->twist.linear.z_depth;
  } else if (_msg->z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_ALTITUDE) {
    _msg->slave.z = odom_->twist.linear.z_altitude;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Z mode(%d) is invalid.", _msg->z_mode);
  }

  is_update = false;
  pub_->publish(std::move(*_msg));
  RCLCPP_DEBUG(this->get_logger(), "Published WrenchPlan");
}

void VelocityWrenchPlanner::odomCallback(const localization_msgs::msg::Odometry::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  is_update = true;
  odom_ = _msg;
}

}  // namespace planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planner::VelocityWrenchPlanner)
