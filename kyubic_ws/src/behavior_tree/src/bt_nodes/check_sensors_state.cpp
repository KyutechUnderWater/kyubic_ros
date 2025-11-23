/**
 * @file check_sensors_state.cpp
 * @brief Check sensors state
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details 各種センサ情報を取得し，エラーが出ているか確認する
 ************************************************************/

#include "behavior_tree/check_sensors_state.hpp"

CheckSensorsState::CheckSensorsState(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::ConditionNode(name, config), ros_node_(ros_node), imu_state_(2), depth_state_(2)
{
  imu_sub_ = ros_node_->create_subscription<driver_msgs::msg::IMU>(
    "/driver/imu", 10,
    [this](driver_msgs::msg::IMU::SharedPtr msg) { this->imu_state_ = msg->status.id; });

  depth_sub_ = ros_node_->create_subscription<driver_msgs::msg::Depth>(
    "/driver/depth", 10,
    [this](driver_msgs::msg::Depth::SharedPtr msg) { this->depth_state_ = msg->status.id; });
}

BT::PortsList CheckSensorsState::providedPorts() { return {}; }

BT::NodeStatus CheckSensorsState::tick()
{
  // TODO: データが来ていないときにFAILUREを出力
  // TODO: エラーコードをblackboardに保存するようにする

  if (imu_state_ == 2 || depth_state_ == 2) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}
