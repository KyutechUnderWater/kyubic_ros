/**
 * @file waypoint_action.hpp
 * @brief Action client for PDLA Planner
 * @author S.Itozono
 * @date 2025/11/19
 *
 * @details PDLA Action ServerにCSVパスを送り、経路追従を実行するBTノード
 ***********************************************************************/

#ifndef _WAYPOINT_ACTION_HPP
#define _WAYPOINT_ACTION_HPP

#include <planner_msgs/action/pdla.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "behavior_tree/common/ros_action_node.hpp"

namespace behavior_tree
{

/**
 * @brief Waypoint Action Client implementation using the CUSTOM RosActionNode template.
 */
class WaypointAction : public RosActionNode<planner_msgs::action::PDLA>
{
public:
  /**
   * @brief Constructor matching the custom base class signature
   */
  WaypointAction(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub,
    rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  bool setGoal(typename Action::Goal & goal) override;

  BT::NodeStatus onResult(const WrappedResult & wr) override;

  void onFeedback(const std::shared_ptr<const Feedback> feedback) override;

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub_;
};

}  // namespace behavior_tree

#endif  // _WAYPOINT_ACTION_HPP
