#ifndef RETURN_MISSION_PLANNER__RETURN_PLANNER_HPP_
#define RETURN_MISSION_PLANNER__RETURN_PLANNER_HPP_

#include <localization_msgs/msg/odometry.hpp>
#include <mutex>
#include <planner_msgs/action/return.hpp>
#include <planner_msgs/msg/wrench_plan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace planner
{

class ReturnPlanner : public rclcpp::Node
{
public:
  using ReturnAction = planner_msgs::action::Return;
  using GoalHandleReturn = rclcpp_action::ServerGoalHandle<ReturnAction>;

  explicit ReturnPlanner(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<planner_msgs::msg::WrenchPlan>::SharedPtr pub_plan_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp_action::Server<ReturnAction>::SharedPtr action_server_;

  localization_msgs::msg::Odometry::SharedPtr current_odom_;
  std::mutex odom_mutex_;

  void odometryCallback(const localization_msgs::msg::Odometry::SharedPtr msg);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ReturnAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleReturn> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleReturn> goal_handle);

  void execute(const std::shared_ptr<GoalHandleReturn> goal_handle);
};

}  // namespace planner

#endif  // RETURN_MISSION_PLANNER__RETURN_PLANNER_HPP_