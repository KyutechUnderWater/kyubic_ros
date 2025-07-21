/**
 * @file wrench_planner.cpp
 * @brief plan force and torque from path
 * @author R.Ohnishi
 * @date 2025/07/15
 *
 * @details pathからforceとtorqeを計画
 *************************************/

#include "wrench_planner/wrench_planner.hpp"

#include <rclcpp/logging.hpp>

#include "geometry_msgs/msg/wrench_stamped.hpp"

#include <functional>
#include <memory>
#include <utility>

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
  sub_ = create_subscription<planner_msgs::msg::WrenchPlan>(
    "goal_current_odom", qos,
    std::bind(&WrenchPlanner::goalCurrentOdomCallback, this, std::placeholders::_1));
}

void WrenchPlanner::_update_wrench()
{
  const uint8_t & z_mode = goal_current_odom_->z_mode;
  const auto & target_pose = goal_current_odom_->target;
  const auto & current_pose = goal_current_odom_->odom.pose;
  const auto & current_twst = goal_current_odom_->odom.twist;

  auto msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();

  if (goal_current_odom_->odom.status == localization_msgs::msg::Odometry::STATUS_ERROR) {
    RCLCPP_ERROR(this->get_logger(), "The current odometry is invalid");
  } else {
    double force_x = p_pid_ctrl_->pid_x_update(
      current_twst.linear.x, current_pose.position.x, target_pose.position.x);
    double force_y = p_pid_ctrl_->pid_y_update(
      current_twst.linear.y, current_pose.position.y, target_pose.position.y);

    double force_z = 0.0;
    if (z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH) {
      force_z = p_pid_ctrl_->pid_z_update(
        current_twst.linear.z_depth, current_pose.position.z_depth, target_pose.position.z_depth);
    } else if (z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_ALTITUDE) {
      force_z = p_pid_ctrl_->pid_z_update(
        current_twst.linear.z_altitude, current_pose.position.z_altitude,
        target_pose.position.z_altitude);
    } else {
      RCLCPP_ERROR(this->get_logger(), "z_mode is failure");
      return;
    }

    double torque_x = p_pid_ctrl_->pid_roll_update(
      current_twst.angular.x, current_pose.orientation.x, target_pose.orientation.x);

    double torque_z = p_pid_ctrl_->pid_yaw_update(
      current_twst.angular.z, current_pose.orientation.z, target_pose.orientation.z);

    // z-axis transform
    double z_rad = -current_pose.orientation.z * std::numbers::pi / 180;
    double _force_x = force_x;
    double _force_y = force_y;
    force_x = _force_x * cos(z_rad) - _force_y * sin(z_rad);
    force_y = _force_x * sin(z_rad) + _force_y * cos(z_rad);

    RCLCPP_DEBUG(
      this->get_logger(), "P-PID -> x: %f, y: %f, z: %f, roll: %f, yaw: %f", force_x, force_y,
      force_z, torque_x, torque_z);

    {
      msg->wrench.force.x = force_x;
      msg->wrench.force.y = force_y;
      msg->wrench.force.z = force_z;
      msg->wrench.torque.x = torque_x;
      msg->wrench.torque.z = torque_z;
    }
  }
  pub_->publish(std::move(msg));
}

void WrenchPlanner::goalCurrentOdomCallback(const planner_msgs::msg::WrenchPlan::SharedPtr msg)
{
  goal_current_odom_ = msg;
  _update_wrench();
}

}  // namespace planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planner::WrenchPlanner)
