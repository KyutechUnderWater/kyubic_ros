#ifndef _PDLA_PLANNER_HPP
#define _PDLA_PLANNER_HPP

#include <cstdint>
#include <localization_msgs/msg/odometry.hpp>
#include <mutex>
#include <path_planner/path_csv_loader.hpp>
#include <planner_msgs/action/pdla.hpp>
#include <planner_msgs/msg/wrench_plan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <timer/timeout.hpp>

#include "planner_msgs/action/pdla.hpp"

namespace planner::pdla_planner
{

struct Tolerance
{
  double x;
  double y;
  double z;
  double roll;
  double yaw;
};

class PDLAPlanner : public rclcpp::Node
{
public:
  explicit PDLAPlanner(const rclcpp::NodeOptions & options);

private:
  uint64_t fine_timer_ms_;
  double look_ahead_scale;
  Tolerance reach_tolerance, waypoint_tolerance;

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::Publisher<planner_msgs::msg::WrenchPlan>::SharedPtr pub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp_action::Server<planner_msgs::action::PDLA>::SharedPtr action_server_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<timer::Timeout> timeout_;

  // Action Callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const planner_msgs::action::PDLA::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<planner_msgs::action::PDLA>> goal_handle);
  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<planner_msgs::action::PDLA>> goal_handle);

  void odometryCallback(const localization_msgs::msg::Odometry::SharedPtr msg);

  void _runPlannerLogic(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<planner_msgs::action::PDLA>> &
      goal_handle);
  bool _checkReached(
    PoseData & target, localization_msgs::msg::Odometry::SharedPtr & odom_copy, Tolerance tol);
  void _print_waypoint(std::string label, size_t step_idx);

  void timerCallback();

  bool fine_flag_ = false;
  std::vector<PoseData> target_pose_;

  bool last_reached_ = false;
  rclcpp::Time fine_reached_time_;

  bool first_reached_ = true;
  rclcpp::Time first_reached_time_;

  std::string file_path_;
  uint8_t step_state_ = planner_msgs::action::PDLA::Feedback::RUNNING;

  std::mutex odom_mutex_;
  std::mutex goal_mutex_;
  std::shared_ptr<localization_msgs::msg::Odometry> current_odom_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<planner_msgs::action::PDLA>> active_goal_handle_;

  size_t step_idx = 0;
  rclcpp::Time step_start_time_;
  double waypoint_timeout_sec_ = 0.0;
};

}  // namespace planner::pdla_planner

#endif  // !_PDLA_PLANNER_HPP
