/**
 * @file find_pinger_action.hpp
 * @brief Action client for SBL (Find Pinger)
 * @details Triggers the acoustic pinger search on Raspberry Pi
 *************************************************************/

#ifndef _FIND_PINGER_ACTION_HPP
#define _FIND_PINGER_ACTION_HPP

#include <planner_msgs/action/find_pinger.hpp>

#include "behavior_tree/common/ros_action_node.hpp"

namespace behavior_tree
{

/**
 * @brief FindPinger Action Client implementation
 * Target Action: planner_msgs::action::FindPinger
 */
class FindPingerAction : public RosActionNode<planner_msgs::action::FindPinger>
{
public:
  FindPingerAction(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  bool setGoal(typename Action::Goal & goal) override;

  BT::NodeStatus onResult(const WrappedResult & wr) override;

  void onFeedback(const std::shared_ptr<const Feedback> feedback) override;
};

}  // namespace behavior_tree

#endif  // _FIND_PINGER_ACTION_HPP