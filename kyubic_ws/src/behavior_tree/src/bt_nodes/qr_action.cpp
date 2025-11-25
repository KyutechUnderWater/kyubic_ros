/**
 * @file qr_action.hpp
 * @brief Action client for QR tracking
 * @author S.Itozono
 * @date 2025/11/22
 *
 * @details QRコードトラッキングのAction ClientのBTノード
 *******************************************************/

#include "behavior_tree/qr_action.hpp"

namespace behavior_tree
{

QrAction::QrAction(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub, rclcpp::Node::SharedPtr ros_node)
: RosActionNode(name, config, ros_node), logger_pub_(logger_pub)
{
}

BT::PortsList QrAction::providedPorts()
{
  return {BT::InputPort<std::string>("action_name", "qr_action", "Action server name")};
}

bool QrAction::setGoal(planner_msgs::action::QR::Goal & goal)
{
  goal.start = true;

  RCLCPP_DEBUG(
    ros_node_->get_logger(), "QrAction: Sending Goal (start=true) to %s", action_name.c_str());
  return true;
}

BT::NodeStatus QrAction::onResult(const WrappedResult & wr)
{
  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result->success) {
    RCLCPP_DEBUG(ros_node_->get_logger(), "QrAction: Succeeded");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "QrAction: Failed (Code: %d)", (int)wr.code);
    return BT::NodeStatus::SUCCESS;
  }
}

// フィードバック受信時の処理
void QrAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  std::string s = std::format(
    "QR Feedback: [x: {:.2f}, y: {:.2f}, z: {:.2f}, conf: {:.2f}]", feedback->x, feedback->y,
    feedback->z, feedback->confidence);

  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "[QrAction] " + s;

  logger_pub_->publish(std::move(msg));
  RCLCPP_INFO_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 1000, "%s", s.c_str());
}

}  // namespace behavior_tree
