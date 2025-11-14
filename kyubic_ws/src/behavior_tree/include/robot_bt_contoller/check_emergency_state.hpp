#ifndef ROBOT_BT_CONTROLLER__CHECK_EMERGENCY_STATE_HPP_
#define ROBOT_BT_CONTROLLER__CHECK_EMERGENCY_STATE_HPP_

#include <atomic>

#include "behaviortree_cpp/condition_node.h"
#include "custom_msgs/msg/depth_state.hpp"  // <-- 仮定
#include "custom_msgs/msg/imu_state.hpp"    // <-- 仮定
#include "rclcpp/rclcpp.hpp"

class CheckEmergencyState : public BT::ConditionNode
{
public:
  CheckEmergencyState(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config), imu_state_(0), depth_state_(0)
  {
    // BT ManagerノードのポインタをBlackboardから取得
    auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    imu_sub_ = node->create_subscription<custom_msgs::msg::ImuState>(
      "/imu_state", 10,  // <-- トピック名は要確認
      [this](custom_msgs::msg::ImuState::SharedPtr msg) { this->imu_state_ = msg->state; });

    depth_sub_ = node->create_subscription<custom_msgs::msg::DepthState>(
      "/depth_state", 10,  // <-- トピック名は要確認
      [this](custom_msgs::msg::DepthState::SharedPtr msg) { this->depth_state_ = msg->state; });
  }

  static BT::PortsList providedPorts()
  {
    return {};  // このノードはXMLから入力ポートを必要としない
  }

  // メインの実行ロジック
  BT::NodeStatus tick() override
  {
    if (imu_state_ == 2 || depth_state_ == 2) {
      return BT::NodeStatus::SUCCESS;  // 緊急状態
    }
    return BT::NodeStatus::FAILURE;  // 正常
  }

private:
  rclcpp::Subscription<custom_msgs::msg::ImuState>::SharedPtr imu_sub_;
  rclcpp::Subscription<custom_msgs::msg::DepthState>::SharedPtr depth_sub_;
  std::atomic<uint8_t> imu_state_;
  std::atomic<uint8_t> depth_state_;
};
#endif  // ROBOT_BT_CONTROLLER__CHECK_EMERGENCY_STATE_HPP_