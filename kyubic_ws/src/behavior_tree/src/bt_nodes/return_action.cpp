#include "behavior_tree/return_action.hpp"

namespace behavior_tree
{

ReturnAction::ReturnAction(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: RosActionNode<planner_msgs::action::Return>(name, config, ros_node)
{
}

BT::PortsList ReturnAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("action_name", "return_action", "Action Server Name"),
    BT::InputPort<double>("surface_depth", 0.0, "Target Depth"),
    BT::InputPort<double>("xy_tolerance", 0.5, "XY Tolerance"),
    BT::InputPort<double>("depth_tolerance", 0.2, "Depth Tolerance")};
}

bool ReturnAction::setGoal(typename Action::Goal & goal)
{
  goal.surface_depth = getInput<double>("surface_depth").value_or(0.0);
  goal.xy_tolerance = getInput<double>("xy_tolerance").value_or(0.5);
  goal.depth_tolerance = getInput<double>("depth_tolerance").value_or(0.2);
  return true;
}

BT::NodeStatus ReturnAction::onResult(const WrappedResult & wr)
{
  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(
      ros_node_->get_logger(), "[%s] Task Succeeded: %s", name().c_str(),
      wr.result->message.c_str());
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "[%s] Task Failed or Canceled", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
}

void ReturnAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO_THROTTLE(
    ros_node_->get_logger(), *ros_node_->get_clock(), 2000, "[%s] State: %s, Dist: %.2fm",
    name().c_str(), feedback->state.c_str(), feedback->distance_to_target);
}

}  // namespace behavior_tree