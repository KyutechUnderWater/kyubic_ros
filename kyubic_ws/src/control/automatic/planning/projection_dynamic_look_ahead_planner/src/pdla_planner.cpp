/**
 * @file PDLA_planner.cpp
 * @brief Tracking through the pass with Projection Dynamic Look-Ahead Planner
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details ベクトル射影を用いた動的前方注視制御による経路追従
 ****************************************************************************/

#include "projection_dynamic_look_ahead_planner/pdla_planner.hpp"

#include <Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

#include <cstddef>
#include <functional>
#include <memory>

namespace planner
{

PDLAPlanner::PDLAPlanner(const rclcpp::NodeOptions & options) : Node("pdla_planner", options)
{
  std::string path_planner_path = this->declare_parameter("path_planner_path", "");
  std::string path_csv = this->declare_parameter("path_csv", "");
  look_ahead_scale = this->declare_parameter("look_ahead_scale", 1.0);

  reach_tolerance.x = this->declare_parameter("reach_tolerance.x", 0.0);
  reach_tolerance.y = this->declare_parameter("reach_tolerance.y", 0.0);
  reach_tolerance.z = this->declare_parameter("reach_tolerance.z", 0.0);
  reach_tolerance.roll = this->declare_parameter("reach_tolerance.roll", 0.0);
  reach_tolerance.yaw = this->declare_parameter("reach_tolerance.yaw", 0.0);

  waypoint_tolerance.x = this->declare_parameter("waypoint_tolerance.x", 0.0);
  waypoint_tolerance.y = this->declare_parameter("waypoint_tolerance.y", 0.0);
  waypoint_tolerance.z = this->declare_parameter("waypoint_tolerance.z", 0.0);
  waypoint_tolerance.roll = this->declare_parameter("waypoint_tolerance.roll", 0.0);
  waypoint_tolerance.yaw = this->declare_parameter("waypoint_tolerance.yaw", 0.0);

  // Load csv file with path
  PathCsvLoader loader;
  loader.parse(path_planner_path + '/' + path_csv);
  std::shared_ptr<PathData> path = loader.get_data();

  // Read target pose
  if (path->get_params().catmull_rom) {
    target_pose_ = path->get_catmulls();
  } else {
    target_pose_ = path->get_checkpoints();
  }
  _print_waypoint("Current", 0);
  _print_waypoint("Next   ", 1);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  pub_ = create_publisher<planner_msgs::msg::WrenchPlan>("goal_current_odom", qos);
  sub_ = create_subscription<localization_msgs::msg::Odometry>(
    "odom", qos, std::bind(&PDLAPlanner::odometryCallback, this, std::placeholders::_1));
}

void PDLAPlanner::_updateGoal()
{
  if (step_idx == target_pose_.size() - 1) {
    if (_checkReached(target_pose_.at(step_idx), reach_tolerance)) {
      RCLCPP_INFO(this->get_logger(), "Reached end point!");
      return;
    }
  } else {
    if (_checkReached(target_pose_.at(step_idx), waypoint_tolerance)) {
      step_idx++;
      RCLCPP_INFO(this->get_logger(), "Reached %d/%lu waypoint.", step_idx, target_pose_.size());
      _print_waypoint("Current", step_idx);

      if (step_idx < target_pose_.size() - 1) {
        _print_waypoint("Next   ", step_idx + 1);
      }
    }
  }

  Eigen::Vector3d virtual_goal_point;
  if (step_idx == 0) {
    virtual_goal_point << target_pose_.at(0).x, target_pose_.at(0).y, target_pose_.at(0).z;
  } else if (step_idx == target_pose_.size() - 1) {
    virtual_goal_point << target_pose_[step_idx].x, target_pose_[step_idx].y,
      target_pose_[step_idx].z;
  } else {
    PoseData pre_step_pose = target_pose_.at(step_idx - 1);
    PoseData current_step_pose = target_pose_.at(step_idx);
    PoseData next_step_pose = target_pose_.at(step_idx + 1);
    Eigen::Vector3d p1(pre_step_pose.x, pre_step_pose.y, pre_step_pose.z);
    Eigen::Vector3d p2(current_step_pose.x, current_step_pose.y, current_step_pose.z);
    Eigen::Vector3d p3(next_step_pose.x, next_step_pose.y, next_step_pose.z);

    localization_msgs::msg::Pose current_pose = current_odom_->pose;
    Eigen::Vector3d p0;
    if (current_step_pose.z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH) {
      p0 << current_pose.position.x, current_pose.position.y, current_pose.position.z_depth;
    } else if (current_step_pose.z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_ALTITUDE) {
      p0 << current_pose.position.x, current_pose.position.y, current_pose.position.z_altitude;
    } else {
      RCLCPP_FATAL(this->get_logger(), "z mode (%d) is failer.", current_step_pose.z_mode);
      rclcpp::shutdown();
    }

    Eigen::Vector3d v02 = p2 - p0;
    Eigen::Vector3d v12 = p2 - p1;
    Eigen::Vector3d v23 = p3 - p2;

    // ±90度以上の方向転換，移動なしのとき，仮想目標点を実際の目標点にする
    if (v02.dot(v23) <= 0 || v12.norm() == 0) {
      virtual_goal_point = p2;
    }
    // 仮想目標点の計算
    else {
      v02 = v02 * v12.norm() / v02.norm();
      Eigen::Vector3d proj_v02_v23 = v02 * v02.dot(v23) / v02.norm();
      virtual_goal_point = (p2 - v02) + (v02 + proj_v02_v23) * look_ahead_scale;
    }
  }

  {
    auto msg = std::make_unique<planner_msgs::msg::WrenchPlan>();

    msg->header.stamp = this->get_clock()->now();
    msg->z_mode = target_pose_.at(step_idx).z_mode;
    msg->target.position.x = virtual_goal_point.x();
    msg->target.position.y = virtual_goal_point.y();

    if (target_pose_.at(step_idx).z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH) {
      msg->target.position.z_depth = virtual_goal_point.z();
    } else if (target_pose_.at(step_idx).z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_ALTITUDE) {
      msg->target.position.z_altitude = virtual_goal_point.z();
    }

    msg->target.orientation.x = target_pose_.at(step_idx).roll;
    msg->target.orientation.z = target_pose_.at(step_idx).yaw;
    msg->odom = *current_odom_;

    pub_->publish(std::move(msg));
  }
}

bool PDLAPlanner::_checkReached(PoseData & target, Tolerance tolerance)
{
  double current_z;
  if (target.z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH) {
    current_z = current_odom_->pose.position.z_depth;
  } else {
    current_z = current_odom_->pose.position.z_altitude;
  }

  if (
    abs(target.x - current_odom_->pose.position.x) < tolerance.x &&
    abs(target.y - current_odom_->pose.position.y) < tolerance.y &&
    abs(target.z - current_z) < tolerance.z &&
    abs(target.roll - current_odom_->pose.orientation.x) < tolerance.roll &&
    abs(target.yaw - current_odom_->pose.orientation.z) < tolerance.yaw) {
    return true;
  }
  return false;
}

void PDLAPlanner::_print_waypoint(std::string label, size_t step_idx)
{
  PoseData pose = target_pose_.at(step_idx);
  RCLCPP_INFO(
    this->get_logger(),
    "%s %lu/%lu waypoint. x: %7.3f  y: %7.3f  z: %7.3f  z_mode: %u  roll: %7.1f  yaw: %6.1f",
    label.c_str(), step_idx + 1, target_pose_.size(), pose.x, pose.y, pose.z, pose.z_mode,
    pose.roll, pose.yaw);
}

void PDLAPlanner::odometryCallback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = msg;
  _updateGoal();
}

}  // namespace planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planner::PDLAPlanner)
