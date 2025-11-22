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

  srv_reset_ = create_service<std_srvs::srv::Trigger>(
    "~/reset_trigger",
    std::bind(&PDLAPlanner::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "PDLA Action Server started. Waiting for goal...");
}

void PDLAPlanner::reset_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (!current_odom_) {
    response->success = false;
    response->message = "No odometry received yet.";
    RCLCPP_WARN(this->get_logger(), "Cannot reset: No odometry data.");
    return;
  }

  // 現在の値をオフセットとして保存
  offset_x_ = current_odom_->pose.position.x;
  offset_y_ = current_odom_->pose.position.y;
  offset_z_depth_ = current_odom_->pose.position.z_depth;
  offset_yaw_ = current_odom_->pose.orientation.z;

  response->success = true;
  response->message = "PDLA Planner origin reset.";
  RCLCPP_INFO(
    this->get_logger(), "Origin Reset -> X:%.2f, Y:%.2f, Depth:%.2f, Yaw:%.2f", offset_x_,
    offset_y_, offset_z_depth_, offset_yaw_);
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
  std::shared_ptr<localization_msgs::msg::Odometry> odom_copy;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!current_odom_) {
      RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for first odometry message...");
      return;
    }
    // 生データをコピー
    odom_copy = std::make_shared<localization_msgs::msg::Odometry>(*current_odom_);
  }

  // 1. 位置のオフセット適用 (原点を0に移動)
  odom_copy->pose.position.x -= offset_x_;
  odom_copy->pose.position.y -= offset_y_;
  odom_copy->pose.position.z_depth -= offset_z_depth_;
  odom_copy->pose.orientation.z -= offset_yaw_;

  // 2. 速度ベクトルの回転 
  double global_vx = odom_copy->twist.linear.x;
  double global_vy = odom_copy->twist.linear.y;

  double theta_rad = -offset_yaw_ * std::numbers::pi / 180.0;

  odom_copy->twist.linear.x = global_vx * cos(theta_rad) - global_vy * sin(theta_rad);
  odom_copy->twist.linear.y = global_vx * sin(theta_rad) + global_vy * cos(theta_rad);

  bool reached = false;
  PoseData target = target_pose_.at(step_idx);
  Tolerance tol = (step_idx == target_pose_.size() - 1) ? reach_tolerance : waypoint_tolerance;

  double current_z_val = (target.z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH)
                           ? odom_copy->pose.position.z_depth
                           : odom_copy->pose.position.z_altitude;

  if (
    abs(target.x - odom_copy->pose.position.x) < tol.x &&
    abs(target.y - odom_copy->pose.position.y) < tol.y && abs(target.z - current_z_val) < tol.z &&
    abs(target.roll - odom_copy->pose.orientation.x) < tol.roll &&
    abs(target.yaw - odom_copy->pose.orientation.z) < tol.yaw) {
    reached = true;
  }

  if (reached) {
    if (step_idx == target_pose_.size() - 1) {
      RCLCPP_INFO(this->get_logger(), "Reached end point!");
      auto result = std::make_shared<planner_msgs::action::PDLA::Result>();
      result->success = true;
      goal_handle->succeed(result);
      std::lock_guard<std::mutex> lock(goal_mutex_);
      active_goal_handle_.reset();
      return;
    } else {
      step_idx++;
      RCLCPP_INFO(this->get_logger(), "Reached %zu/%lu waypoint.", step_idx, target_pose_.size());
      _print_waypoint("Current", step_idx);
      if (step_idx < target_pose_.size() - 1) {
        _print_waypoint("Next   ", step_idx + 1);
      }
    }
  }

  // 仮想目標点の計算 (座標変換後の odom_copy を使用)
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

    // [修正] current_pose は補正済みの odom_copy から取得
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

    msg->targets.x = virtual_goal_point.x();
    msg->targets.y = virtual_goal_point.y();
    msg->targets.z = virtual_goal_point.z();
    msg->targets.roll = target_pose_.at(step_idx).roll;
    msg->targets.yaw = target_pose_.at(step_idx).yaw;

    // WrenchPlannerに送るMasterの値も、補正済みの値を入れる
    // これでWrenchPlannerは「ロボットは(0,0)付近にいる」と認識し、
    // 目標点(CSVも0,0付近)との偏差を正しく計算できる
    msg->master.x = odom_copy->pose.position.x;
    msg->master.y = odom_copy->pose.position.y;
    msg->master.roll = odom_copy->pose.orientation.x;
    msg->master.yaw = odom_copy->pose.orientation.z;

    msg->slave.x = odom_copy->twist.linear.x;
    msg->slave.y = odom_copy->twist.linear.y;
    msg->slave.roll = odom_copy->twist.angular.x;
    msg->slave.yaw = odom_copy->twist.angular.z;

    if (target_pose_.at(step_idx).z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH) {
      msg->master.z = odom_copy->pose.position.z_depth;
      msg->slave.z = odom_copy->twist.linear.z_depth;
    } else {
      msg->master.z = odom_copy->pose.position.z_altitude;
      msg->slave.z = odom_copy->twist.linear.z_altitude;
    }
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
