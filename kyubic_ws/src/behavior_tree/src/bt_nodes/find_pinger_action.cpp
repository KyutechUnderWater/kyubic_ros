/**
 * @file find_pinger_action.cpp
 * @brief Action client for SBL (Find Pinger)
 *************************************************************/

#include "behavior_tree/find_pinger_action.hpp"

namespace behavior_tree
{

FindPingerAction::FindPingerAction(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: RosActionNode(name, config, ros_node)
{
}

BT::PortsList FindPingerAction::providedPorts()
{
  return {
    // sbl_node.py で指定されているアクション名は 'find_pinger'
    BT::InputPort<std::string>("action_name", "find_pinger", "Action server name"),
  };
}

bool FindPingerAction::setGoal(planner_msgs::action::FindPinger::Goal & goal)
{
  // ゴールは bool start のみ
  goal.start = true;

  RCLCPP_INFO(
    ros_node_->get_logger(), "FindPingerAction: Sending Goal (start=true) to %s",
    action_name.c_str());
  return true;
}

BT::NodeStatus FindPingerAction::onResult(const WrappedResult & wr)
{
  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result->success) {
    RCLCPP_INFO(
      ros_node_->get_logger(), "FindPingerAction: Succeeded. Final Yaw: %.2f",
      wr.result->final_yaw);
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "FindPingerAction: Failed (Code: %d)", (int)wr.code);
    return BT::NodeStatus::SUCCESS;
  }
}

void FindPingerAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO_THROTTLE(
    ros_node_->get_logger(), *ros_node_->get_clock(), 2000,
    "FindPinger Feedback: Yaw %.2f, Score %.2f", feedback->current_yaw, feedback->current_score);
}

}  // namespace behavior_tree