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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstddef>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

namespace planner::pdla_planner
{

PDLAPlanner::PDLAPlanner(const rclcpp::NodeOptions & options) : Node("pdla_planner", options)
{
  fine_timer_ms_ = this->declare_parameter("fine_timer_ms", 1000);
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

  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), 0);
  timer_ = create_wall_timer(500ms, std::bind(&PDLAPlanner::timerCallback, this));

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

  if (this->timer_->is_canceled()) this->timer_->reset();

  // Load the CSV file(waypoint)
  file_path_ = goal->csv_file_path;
  if (!file_path_.empty() && file_path_[0] != '/') {
    try {
      std::string package_share_dir = ament_index_cpp::get_package_share_directory("path_planner");

      // パスを結合 (shareディレクトリ + XMLに書かれた相対パス)
      file_path_ = package_share_dir + "/" + file_path_;

      RCLCPP_INFO(this->get_logger(), "Resolved relative path to: %s", file_path_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to resolve package path: %s", e.what());
    }
  }

  PathCsvLoader loader;
  try {
    loader.parse(file_path_);
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

  // Set waypoint action timeout
  timeout_->set_timeout(path->get_params().timeout_sec * 1e9);
  timeout_->reset(this->get_clock()->now());

  // Reset variables
  this->step_idx = 0;
  this->last_reached_ = false;
  this->first_reached_ = true;
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
    odom_copy = std::make_shared<localization_msgs::msg::Odometry>(*current_odom_);
  }

  if (
    odom_copy->status.depth.id == common_msgs::msg::Status::ERROR ||
    odom_copy->status.imu.id == common_msgs::msg::Status::ERROR ||
    odom_copy->status.dvl.id == common_msgs::msg::Status::ERROR) {
    RCLCPP_ERROR(this->get_logger(), "The current odometry is invalid");

    // Control when some sensors become unusable
    auto msg = std::make_unique<planner_msgs::msg::WrenchPlan>();
    msg->header.stamp = this->get_clock()->now();
    msg->z_mode = target_pose_.at(step_idx).z_mode;

    if (
      odom_copy->status.depth.id != common_msgs::msg::Status::ERROR &&
      target_pose_.at(step_idx).z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH) {
      msg->targets.z = target_pose_.at(step_idx).z;
      msg->master.z = odom_copy->pose.position.z_depth;
      msg->slave.z = odom_copy->twist.linear.z_depth;
    } else if (
      odom_copy->status.dvl.id != common_msgs::msg::Status::ERROR &&
      target_pose_.at(step_idx).z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_ALTITUDE) {
      msg->targets.z = target_pose_.at(step_idx).z;
      msg->master.z = odom_copy->pose.position.z_altitude;
      msg->slave.z = odom_copy->twist.linear.z_altitude;
    }

    if (odom_copy->status.imu.id != common_msgs::msg::Status::ERROR) {
      msg->targets.roll = target_pose_.at(step_idx).roll;
      msg->targets.yaw = target_pose_.at(step_idx).yaw;

      msg->master.roll = odom_copy->pose.orientation.x;
      msg->master.yaw = odom_copy->pose.orientation.z;

      msg->slave.roll = odom_copy->twist.angular.x;
      msg->slave.yaw = odom_copy->twist.angular.z;
    }

    if (odom_copy->status.dvl.id != common_msgs::msg::Status::ERROR) {
      msg->targets.x = target_pose_.at(step_idx).x;
      msg->targets.y = target_pose_.at(step_idx).y;

      msg->master.x = odom_copy->pose.position.x;
      msg->master.y = odom_copy->pose.position.y;

      msg->slave.x = odom_copy->twist.linear.x;
      msg->slave.y = odom_copy->twist.linear.y;
    }

    {
      pub_->publish(std::move(msg));
      RCLCPP_WARN(this->get_logger(), "Constrained Control (Sensor Error)");
    }
  } else {
    // ウェイポイント到達判定
    bool reached = false;
    PoseData target = target_pose_.at(step_idx);
    Tolerance tol = (step_idx == target_pose_.size() - 1) ? reach_tolerance : waypoint_tolerance;

    if (_checkReached(target, odom_copy, tol)) {
      auto now = this->get_clock()->now();

      if (first_reached_) {
        first_reached_ = false;
        first_reached_time_ = now;
      }

      // If wait time is set
      if ((now - first_reached_time_).nanoseconds() + 1 > target.wait_ms * 1e6)
        first_reached_ = reached = true;
      else
        step_state_ = planner_msgs::action::PDLA::Feedback::WAITING;
    }

    if (reached) {
      timeout_->reset(this->get_clock()->now());

      if (step_idx == target_pose_.size() - 1) {
        RCLCPP_INFO(this->get_logger(), "Reached end point!");
        auto result = std::make_shared<planner_msgs::action::PDLA::Result>();
        result->success = true;
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (active_goal_handle_ == goal_handle && goal_handle->is_active()) {
          goal_handle->succeed(result);
          active_goal_handle_.reset();
        }
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

    // 仮想目標点の計算
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

      msg->master.x = odom_copy->pose.position.x;
      msg->master.y = odom_copy->pose.position.y;
      msg->master.roll = odom_copy->pose.orientation.x;
      msg->master.yaw = odom_copy->pose.orientation.z;

      msg->has_slave = true;
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
  }

  // Feedbackの送信
  {
    auto feedback = std::make_shared<planner_msgs::action::PDLA::Feedback>();
    feedback->csv_file_path = file_path_;
    feedback->step_idx =
      static_cast<uint32_t>(step_state_ == feedback->REACHED ? step_idx - 1 : step_idx);
    feedback->step_state = step_state_;
    feedback->odom = *odom_copy;

    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (active_goal_handle_ == goal_handle && goal_handle->is_active()) {
      goal_handle->publish_feedback(feedback);
    }
  }
}

bool PDLAPlanner::_checkReached(
  PoseData & target, localization_msgs::msg::Odometry::SharedPtr & odom, Tolerance tol)
{
  double current_z_val = (target.z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH)
                           ? odom->pose.position.z_depth
                           : odom->pose.position.z_altitude;

  // Check if reached target
  if (
    abs(target.x - odom->pose.position.x) < tol.x &&
    abs(target.y - odom->pose.position.y) < tol.y && abs(target.z - current_z_val) < tol.z &&
    abs(target.roll - odom->pose.orientation.x) < tol.roll &&
    abs(target.yaw - odom->pose.orientation.z) < tol.yaw) {
    if (!last_reached_) fine_reached_time_ = this->get_clock()->now();
    last_reached_ = false;

    // If fine mode is on, stop detection when fine timer is over
    if (target.fine) {
      if ((this->get_clock()->now() - fine_reached_time_).nanoseconds() < fine_timer_ms_ * 1e6) {
        step_state_ = planner_msgs::action::PDLA::Feedback::FINE_TUNING;
        last_reached_ = true;
        return false;
      }
    }
    step_state_ = planner_msgs::action::PDLA::Feedback::REACHED;
    return true;
  }
  step_state_ = planner_msgs::action::PDLA::Feedback::RUNNING;
  last_reached_ = false;
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
    "%s %lu/%lu waypoint. x: %7.3f  y: %7.3f  z: %7.3f  z_mode: %u  roll: %7.1f  yaw: %6.1f  "
    "wait_ms: %.0f  fine: %d",
    label.c_str(), step_idx + 1, target_pose_.size(), pose.x, pose.y, pose.z, pose.z_mode,
    pose.roll, pose.yaw, pose.wait_ms, pose.fine);
}

void PDLAPlanner::timerCallback()
{
  std::lock_guard<std::mutex> lock(goal_mutex_);

  if (!active_goal_handle_) {
    this->timer_->cancel();
    return;
  }

  if (active_goal_handle_->is_canceling()) {
    auto result = std::make_shared<planner_msgs::action::PDLA::Result>();
    result->success = false;
    active_goal_handle_->canceled(result);
    active_goal_handle_.reset();

    RCLCPP_WARN(this->get_logger(), "Goal canceled.");
    return;
  }

  if (timeout_->is_timeout(this->get_clock()->now())) {
    auto result = std::make_shared<planner_msgs::action::PDLA::Result>();
    result->success = false;
    active_goal_handle_->abort(result);
    active_goal_handle_.reset();

    RCLCPP_ERROR(this->get_logger(), "Timeout reached (Odom might be lost)! Aborting goal.");
  }
}

}  // namespace planner::pdla_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planner::pdla_planner::PDLAPlanner)
