/**
 * @file qr_action.cpp
 * @brief Action client for QR Planner
 * @details Sends 'start=true' to the qr_planner action server
 *************************************************************/

#include "behavior_tree/qr_action.hpp"

namespace behavior_tree
{

QrAction::QrAction(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: RosActionNode(name, config, ros_node)
{
}

// ポート定義
BT::PortsList QrAction::providedPorts()
{
  return {BT::InputPort<std::string>("action_name", "qr_action", "Action server name")};
}

// ゴール設定
bool QrAction::setGoal(planner_msgs::action::QR::Goal & goal)
{
  goal.start = true;

  RCLCPP_INFO(
    ros_node_->get_logger(), "QrAction: Sending Goal (start=true) to %s", action_name.c_str());
  return true;
}

// 結果受信時の処理
BT::NodeStatus QrAction::onResult(const WrappedResult & wr)
{
  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result->success) {
    RCLCPP_INFO(ros_node_->get_logger(), "QrAction: Succeeded");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "QrAction: Failed (Code: %d)", (int)wr.code);
    return BT::NodeStatus::FAILURE;
  }
}

// フィードバック受信時の処理
void QrAction::onFeedback(const std::shared_ptr<const Feedback> feedback) { (void)feedback; }

}  // namespace behavior_tree