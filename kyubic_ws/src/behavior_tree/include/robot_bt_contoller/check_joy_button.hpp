#ifndef ROBOT_BT_CONTROLLER__CHECK_JOY_BUTTON_HPP_
#define ROBOT_BT_CONTROLLER__CHECK_JOY_BUTTON_HPP_

#include <mutex>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class CheckJoyButton : public BT::ConditionNode
{
public:
  CheckJoyButton(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    joy_sub_ = node->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, [this](sensor_msgs::msg::Joy::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_joy_msg_ = msg;
      });
  }

  static BT::PortsList providedPorts()
  {
    // XMLから "button_index" という名前の入力を受け取る
    return {BT::InputPort<int>("button_index")};
  }

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!last_joy_msg_) {
      return BT::NodeStatus::FAILURE;  // まだJoyメッセージを受信していない
    }

    // XMLからボタンのインデックスを取得
    BT::Expected<int> index = getInput<int>("button_index");
    if (!index) {
      return BT::NodeStatus::FAILURE;  // ポートが設定されていない
    }

    if (
      last_joy_msg_->buttons.size() > index.value() && last_joy_msg_->buttons[index.value()] == 1) {
      return BT::NodeStatus::SUCCESS;  // ボタンが押されている
    }

    return BT::NodeStatus::FAILURE;  // ボタンが押されていない
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  sensor_msgs::msg::Joy::SharedPtr last_joy_msg_;
  std::mutex mutex_;
};
#endif  // ROBOT_BT_CONTROLLER__CHECK_JOY_BUTTON_HPP_