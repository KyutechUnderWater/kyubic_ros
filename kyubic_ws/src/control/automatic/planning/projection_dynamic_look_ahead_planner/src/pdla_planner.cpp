/**
 * @file pdla_planner.cpp
 * @brief Tracking through the pass with Projection Dynamic Look-Ahead Planner (Action Server Ver.)
 * @author R.Ohnishi
 * @date 2025/11/06 (Modified)
 *
 * @details ベクトル射影を用いた動的前方注視制御による経路追従 (ROS 2 Action Server)
 ****************************************************************************/

#include "projection_dynamic_look_ahead_planner/pdla_planner.hpp"

#include <Eigen/Dense>
#include <cstddef>
#include <rclcpp_action/rclcpp_action.hpp>

namespace planner
{

PDLAPlanner::PDLAPlanner(const rclcpp::NodeOptions & options) : Node("pdla_planner", options)
{
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

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  pub_ = create_publisher<planner_msgs::msg::WrenchPlan>("goal_current_odom", qos);

  sub_ = create_subscription<localization_msgs::msg::Odometry>(
    "odom", qos, std::bind(&PDLAPlanner::odometryCallback, this, std::placeholders::_1));

  action_server_ = rclcpp_action::create_server<planner_msgs::action::PDLA>(
    this->get_node_base_interface(), this->get_node_clock_interface(),
    this->get_node_logging_interface(), this->get_node_waitables_interface(), "pdla_plan",
    std::bind(&PDLAPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&PDLAPlanner::handle_cancel, this, std::placeholders::_1),
    std::bind(&PDLAPlanner::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(), callback_group_);

  RCLCPP_INFO(this->get_logger(), "PDLA Action Server started. Waiting for goal...");
}

rclcpp_action::GoalResponse PDLAPlanner::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const planner_msgs::action::PDLA::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(
    this->get_logger(), "Received goal request with path: %s", goal->csv_file_path.c_str());

  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (active_goal_handle_) {
    RCLCPP_WARN(this->get_logger(), "Another goal is already active. Rejecting new goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PDLAPlanner::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<planner_msgs::action::PDLA>> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PDLAPlanner::handle_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<planner_msgs::action::PDLA>> goal_handle)
{
  std::lock_guard<std::mutex> lock(goal_mutex_);

  const auto goal = goal_handle->get_goal();
  PathCsvLoader loader;
  try {
    loader.parse(goal->csv_file_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load or parse CSV file: %s", e.what());
    auto result = std::make_shared<planner_msgs::action::PDLA::Result>();
    result->success = false;
    goal_handle->abort(result);
    return;
  }

  std::shared_ptr<PathData> path = loader.get_data();

  if (path->get_params().catmull_rom) {
    target_pose_ = path->get_catmulls();
  } else {
    target_pose_ = path->get_checkpoints();
  }

  if (target_pose_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No waypoints loaded from CSV. Aborting goal.");
    auto result = std::make_shared<planner_msgs::action::PDLA::Result>();
    result->success = false;
    goal_handle->abort(result);
    return;
  }

  this->step_idx = 0;
  {
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    this->current_odom_.reset();
  }

  this->active_goal_handle_ = goal_handle;

  RCLCPP_INFO(
    this->get_logger(), "Goal accepted. Loaded %lu waypoints. Starting path following.",
    target_pose_.size());
  _print_waypoint("Current", 0);
  _print_waypoint("Next   ", 1);
}

void PDLAPlanner::odometryCallback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_odom_ = msg;
  }

  std::shared_ptr<rclcpp_action::ServerGoalHandle<planner_msgs::action::PDLA>> goal_handle;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    goal_handle = active_goal_handle_;
  }

  if (!goal_handle) return;

  if (goal_handle->is_canceling()) {
    auto result = std::make_shared<planner_msgs::action::PDLA::Result>();
    result->success = false;
    goal_handle->canceled(result);

    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_handle_.reset();
    RCLCPP_INFO(this->get_logger(), "Goal canceled.");
    return;
  }

  _runPlannerLogic(goal_handle);
}

void PDLAPlanner::_runPlannerLogic(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<planner_msgs::action::PDLA>> & goal_handle)
{
  // ウェイポイント到達判定
  if (step_idx == target_pose_.size() - 1) {
    if (_checkReached(target_pose_.at(step_idx), reach_tolerance)) {
      RCLCPP_INFO(this->get_logger(), "Reached end point!");

      auto result = std::make_shared<planner_msgs::action::PDLA::Result>();
      result->success = true;
      goal_handle->succeed(result);

      std::lock_guard<std::mutex> lock(goal_mutex_);
      active_goal_handle_.reset();
      return;
    }
  } else {
    if (_checkReached(target_pose_.at(step_idx), waypoint_tolerance)) {
      step_idx++;
      RCLCPP_INFO(this->get_logger(), "Reached %zu/%lu waypoint.", step_idx, target_pose_.size());
      _print_waypoint("Current", step_idx);

      if (step_idx < target_pose_.size() - 1) {
        _print_waypoint("Next   ", step_idx + 1);
      }
    }
  }

  // 仮想目標点の計算
  Eigen::Vector3d virtual_goal_point;

  std::shared_ptr<localization_msgs::msg::Odometry> odom_copy;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!current_odom_) {
      RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for first odometry message...");
      return;  // オドメトリが来るまで何もしない
    }
    odom_copy = current_odom_;
  }

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

    localization_msgs::msg::Pose current_pose = odom_copy->pose;
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

    if (v02.dot(v23) <= 0 || v12.norm() == 0) {
      virtual_goal_point = p2;
    } else {
      v02 = v02 * v12.norm() / v02.norm();
      Eigen::Vector3d proj_v02_v23 = v02 * v02.dot(v23) / v02.norm();
      virtual_goal_point = (p2 - v02) + (v02 + proj_v02_v23) * look_ahead_scale;
    }
  }

  // 仮想目標点のPublish
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
    msg->odom = *odom_copy;

    pub_->publish(std::move(msg));
  }

  // Feedbackの送信
  {
    auto feedback = std::make_shared<planner_msgs::action::PDLA::Feedback>();
    feedback->current_odom = *odom_copy;
    feedback->current_waypoint_index = static_cast<uint32_t>(step_idx);
    goal_handle->publish_feedback(feedback);
  }
}

bool PDLAPlanner::_checkReached(PoseData & target, Tolerance tolerance)
{
  std::shared_ptr<localization_msgs::msg::Odometry> odom;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!current_odom_) {
      return false;
    }
    odom = current_odom_;
  }

  double current_z;
  if (target.z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH) {
    current_z = odom->pose.position.z_depth;
  } else {
    current_z = odom->pose.position.z_altitude;
  }

  if (
    abs(target.x - odom->pose.position.x) < tolerance.x &&
    abs(target.y - odom->pose.position.y) < tolerance.y &&
    abs(target.z - current_z) < tolerance.z &&
    abs(target.roll - odom->pose.orientation.x) < tolerance.roll &&
    abs(target.yaw - odom->pose.orientation.z) < tolerance.yaw) {
    return true;
  }
  return false;
}

void PDLAPlanner::_print_waypoint(std::string label, size_t step_idx)
{
  if (step_idx >= target_pose_.size()) {
    RCLCPP_WARN(this->get_logger(), "Waypoint index %lu out of bounds.", step_idx);
    return;
  }
  PoseData pose = target_pose_.at(step_idx);
  RCLCPP_INFO(
    this->get_logger(),
    "%s %lu/%lu waypoint. x: %7.3f  y: %7.3f  z: %7.3f  z_mode: %u  roll: %7.1f  yaw: %6.1f",
    label.c_str(), step_idx + 1, target_pose_.size(), pose.x, pose.y, pose.z, pose.z_mode,
    pose.roll, pose.yaw);
}

}  // namespace planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planner::PDLAPlanner)
