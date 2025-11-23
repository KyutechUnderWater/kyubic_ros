#ifndef _RETURN_ACTION_HPP
#define _RETURN_ACTION_HPP

#include <planner_msgs/action/return.hpp>

#include "behavior_tree/common/ros_action_node.hpp"

namespace behavior_tree
{
class ReturnAction : public RosActionNode<planner_msgs::action::Return>
{
public:
  ReturnAction(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  bool setGoal(typename Action::Goal & goal) override;
  BT::NodeStatus onResult(const WrappedResult & wr) override;
  void onFeedback(const std::shared_ptr<const Feedback> feedback) override;
};
}  // namespace behavior_tree
#endif