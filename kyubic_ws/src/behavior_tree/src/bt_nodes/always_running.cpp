/**
 * @file always_running.cpp
 * @brief Behavior Tree Node that always return running
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details 常にBT::NodeStatus::RUNNINGを返すBehaviorTreeのノード
 ****************************************************************/

#include "behavior_tree/always_running.hpp"

#include <memory>

namespace behavior_tree
{

AlwaysRunning::AlwaysRunning(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub)
: BT::StatefulActionNode(name, config), logger_pub_(logger_pub)
{
}

BT::PortsList AlwaysRunning::providedPorts() { return {}; }

BT::NodeStatus AlwaysRunning::onStart()
{
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "[AlwaysRunning] Running";
  logger_pub_->publish(std::move(msg));
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlwaysRunning::onRunning() { return BT::NodeStatus::RUNNING; }

void AlwaysRunning::onHalted() {}

}  // namespace behavior_tree
