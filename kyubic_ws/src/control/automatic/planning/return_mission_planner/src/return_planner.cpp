/**
 * @file return_planner.cpp
 * @brief Action Server for Return (to slightly offset Origin) and Surface task
 *************************************************************/

#include "return_mission_planner/return_planner.hpp"

#include <cmath>
#include <thread>

namespace planner
{

ReturnPlanner::ReturnPlanner(const rclcpp::NodeOptions & options) : Node("return_planner", options)
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));

  pub_plan_ = create_publisher<planner_msgs::msg::WrenchPlan>("goal_current_odom", qos);
  sub_odom_ = create_subscription<localization_msgs::msg::Odometry>(
    "odom", qos, std::bind(&ReturnPlanner::odometryCallback, this, std::placeholders::_1));

  action_server_ = rclcpp_action::create_server<ReturnAction>(
    this, "return_action",
    std::bind(&ReturnPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ReturnPlanner::handle_cancel, this, std::placeholders::_1),
    std::bind(&ReturnPlanner::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Return Planner Action Server started.");
}

void ReturnPlanner::odometryCallback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  current_odom_ = msg;
}

rclcpp_action::GoalResponse ReturnPlanner::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const ReturnAction::Goal>)
{
  RCLCPP_INFO(
    this->get_logger(), "Received return goal request (Target: Offset Origin -0.11,-0.11)");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ReturnPlanner::handle_cancel(const std::shared_ptr<GoalHandleReturn>)
{
  RCLCPP_INFO(this->get_logger(), "Cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ReturnPlanner::handle_accepted(const std::shared_ptr<GoalHandleReturn> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&ReturnPlanner::execute, this, _1), goal_handle}.detach();
}

void ReturnPlanner::execute(const std::shared_ptr<GoalHandleReturn> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing Return to Offset Origin...");
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ReturnAction::Feedback>();
  auto result = std::make_shared<ReturnAction::Result>();

  rclcpp::Rate loop_rate(20);

  // targets
  const double TARGET_X = -0.11;
  const double TARGET_Y = -0.11;

  enum State
  {
    RETURNING,
    SURFACING
  } state = RETURNING;

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Canceled";
      goal_handle->canceled(result);
      return;
    }

    localization_msgs::msg::Odometry current_pose;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!current_odom_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000, "Waiting for odometry...");
        loop_rate.sleep();
        continue;
      }
      current_pose = *current_odom_;
    }

    double dist_xy = std::sqrt(
      std::pow(TARGET_X - current_pose.pose.position.x, 2) +
      std::pow(TARGET_Y - current_pose.pose.position.y, 2));

    // WrenchPlan作成
    auto plan = std::make_unique<planner_msgs::msg::WrenchPlan>();
    plan->header.stamp = this->now();
    plan->z_mode = 0;  // Depth Mode

    // Master
    plan->master.x = current_pose.pose.position.x;
    plan->master.y = current_pose.pose.position.y;
    plan->master.z = current_pose.pose.position.z_depth;
    plan->master.roll = current_pose.pose.orientation.x;
    plan->master.yaw = current_pose.pose.orientation.z;

    plan->slave.x = current_pose.twist.linear.x;
    plan->slave.y = current_pose.twist.linear.y;
    plan->slave.z = current_pose.twist.linear.z_depth;
    plan->slave.roll = current_pose.twist.angular.x;
    plan->slave.yaw = current_pose.twist.angular.z;

    // Target Setup
    plan->targets.roll = 0.0;
    plan->targets.yaw = 0.0;

    if (state == RETURNING) {
      plan->targets.x = TARGET_X;
      plan->targets.y = TARGET_Y;
      plan->targets.z = current_pose.pose.position.z_depth;

      feedback->state = "RETURNING";
      feedback->distance_to_target = dist_xy;

      if (dist_xy < goal->xy_tolerance) {
        RCLCPP_INFO(this->get_logger(), "Reached Offset Origin (XY). Switching to SURFACING.");
        state = SURFACING;
      }
    } else if (state == SURFACING) {
      plan->targets.x = TARGET_X;
      plan->targets.y = TARGET_Y;
      plan->targets.z = goal->surface_depth;

      double dist_z = std::abs(goal->surface_depth - current_pose.pose.position.z_depth);
      feedback->state = "SURFACING";
      feedback->distance_to_target = dist_z;

      if (dist_z < goal->depth_tolerance) {
        RCLCPP_INFO(this->get_logger(), "Surfaced. Task Complete.");
        result->success = true;
        result->message = "Returned to Offset Origin and Surfaced";
        goal_handle->succeed(result);
        return;
      }
    }

    pub_plan_->publish(std::move(plan));
    feedback->current_odom = current_pose;
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }
}

}  // namespace planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planner::ReturnPlanner)