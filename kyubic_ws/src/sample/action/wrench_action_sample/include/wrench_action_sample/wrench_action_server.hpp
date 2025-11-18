#ifndef _WRENCH_ACTION_SERVER_HPP
#define _WRENCH_ACTION_SERVER_HPP

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <wrench_action_sample_msgs/action/wrench.hpp>

#include "rclcpp_action/rclcpp_action.hpp"

/**
 * @brief Action Server node that publishes WrenchStamped messages.
 * @details This server accepts an action goal specifying a duration.
 *          Upon acceptance, it publishes wrench messages to the "robot_force" topic
 *          at 10Hz, linearly increasing the force values based on the iteration count.
 */
class WrenchActionServer : public rclcpp::Node
{
public:
  using Wrench = wrench_action_sample_msgs::action::Wrench;
  using GoalHandleApplyWrench = rclcpp_action::ServerGoalHandle<Wrench>;
  using WrenchMsg = geometry_msgs::msg::WrenchStamped;

  /**
   * @brief Constructor for the WrenchActionServer.
   * @param options Node options for ROS 2 node initialization.
   */
  explicit WrenchActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<Wrench>::SharedPtr action_server_;
  rclcpp::Publisher<WrenchMsg>::SharedPtr publisher_;

  /**
   * @brief Callback to handle a new goal request.
   * @param uuid Unique identifier for the goal.
   * @param goal The goal object containing the requested duration.
   * @return rclcpp_action::GoalResponse Indicates whether the goal is accepted or rejected.
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Wrench::Goal> goal);

  /**
   * @brief Callback to handle a cancellation request.
   * @param goal_handle Handle to the goal being canceled.
   * @return rclcpp_action::CancelResponse Indicates whether the cancel request is accepted.
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleApplyWrench> goal_handle);

  /**
   * @brief Callback executed when a goal has been accepted.
   * @param goal_handle Handle to the active goal.
   * @details Starts the execution thread.
   */
  void handle_accepted(const std::shared_ptr<GoalHandleApplyWrench> goal_handle);

  /**
   * @brief Main execution loop for the action.
   * @param goal_handle Handle to the active goal.
   * @details Publishes WrenchStamped messages at 10Hz until the duration expires or the task is canceled.
   */
  void execute(const std::shared_ptr<GoalHandleApplyWrench> goal_handle);
};

#endif  // WRENCH_ACTION_SERVER_HPP_
