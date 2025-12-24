/**
 * @file talker.cpp
 * @brief Implementation of Talker node
 * @author R.ohnishi
 * @date 2025/12/22
 **************************************************************/

#include "behavior_tree/talker.hpp"

namespace behavior_tree
{

Talker::Talker(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::SyncActionNode(name, config), ros_node_(ros_node)
{
  // デフォルト値またはXML定義からトピック名を取得
  if (!getInput("topic_name", topic_name_)) {
    topic_name_ = "output";  // Default topic name
  }

  // Publisherの作成
  publisher_ = ros_node_->create_publisher<std_msgs::msg::String>(topic_name_, 10);
}

BT::PortsList Talker::providedPorts()
{
  return {
    BT::InputPort<std::string>("message", "Message to publish"),
    BT::InputPort<std::string>("topic_name", "output", "ROS 2 topic name")};
}

BT::NodeStatus Talker::tick()
{
  std::string msg_content;
  if (!getInput("message", msg_content)) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Talker: Missing required input 'message'");
    return BT::NodeStatus::FAILURE;
  }

  if (!publisher_) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Talker: Publisher not initialized");
    return BT::NodeStatus::FAILURE;
  }

  auto message = std_msgs::msg::String();
  message.data = msg_content;
  publisher_->publish(message);

  // オプション: ログ出力が必要な場合
  // RCLCPP_INFO(ros_node_->get_logger(), "Talker published: %s", msg_content.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace behavior_tree
