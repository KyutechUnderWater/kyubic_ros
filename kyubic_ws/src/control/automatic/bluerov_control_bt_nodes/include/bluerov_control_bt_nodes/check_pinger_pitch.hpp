#ifndef _CHECK_PINGER_PITCH_HPP
#define _CHECK_PINGER_PITCH_HPP

#include <behaviortree_cpp/condition_node.h>

#include <planner_msgs/msg/pinger_direction.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief pinger_direction.pitchが閾値以下かを毎tick即座に判定するConditionノード
 * (iwakuni2026.xml、ハイドロフォン追跡ループの「もう十分近いか」のゲート)。
 * @details RUNNINGを返さない(親のFallbackが2番目の子(ステップ移動)を毎tick試せる
 * ようにするため、GoToDepth等のStatefulActionNodeとは異なりConditionNodeとして実装)。
 * pinger_directionを一度も受信していない間はFAILURE(まだ遠いとみなし、安全側に倒す)。
 *
 * 【未検証】pitch<=pitch_threshold_degで「近い」と判定する向き自体は、依頼者が
 * XML側で決めた仕様をそのまま実装したもの。sbl_estimation_node側のpitch正方向の
 * 定義と組み合わせて実機で必ず検証すること。
 */
class CheckPingerPitch : public BT::ConditionNode
{
public:
  CheckPingerPitch(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<planner_msgs::msg::PingerDirection>::SharedPtr pinger_sub_;
  planner_msgs::msg::PingerDirection::SharedPtr latest_pinger_;

  void pingerCallback(const planner_msgs::msg::PingerDirection::SharedPtr msg);
};

}  // namespace bluerov_control_bt_nodes

#endif
