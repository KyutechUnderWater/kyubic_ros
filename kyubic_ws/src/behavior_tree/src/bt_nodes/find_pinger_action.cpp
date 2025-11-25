/**
 * @file find_pinger_action.cpp
 * @brief Action client for SBL (Find Pinger)
 *************************************************************/

#include "behavior_tree/find_pinger_action.hpp"

namespace behavior_tree
{

FindPingerAction::FindPingerAction(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub, rclcpp::Node::SharedPtr ros_node)
: RosActionNode(name, config, ros_node), logger_pub_(logger_pub)
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
  std::string s = std::format(
    "FindPinger Feedback: Yaw {:.2f}, Score {:.2f}", feedback->current_yaw,
    feedback->current_score);

  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "[FindPingerAction] " + s;

  logger_pub_->publish(std::move(msg));
  RCLCPP_INFO_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 2000, "%s", s.c_str());
}

}  // namespace behavior_tree
