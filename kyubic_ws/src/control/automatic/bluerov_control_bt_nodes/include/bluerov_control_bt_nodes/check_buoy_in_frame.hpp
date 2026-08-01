#ifndef _CHECK_BUOY_IN_FRAME_HPP
#define _CHECK_BUOY_IN_FRAME_HPP

#include <behaviortree_cpp/condition_node.h>

#include <buoy_interfaces/msg/buoy_relative_position.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief 画像処理班の生データ(buoy_interfaces/msg/BuoyRelativePosition)を直接購読し、
 * ブイがカメラに写っているかを毎tick即座に判定するConditionノード(iwakuni2026.xml、
 * `Final Descend Successful`直後の「写っていなければハイドロフォン方向へ向き直る」ループ)。
 * @details `CheckPingerPitch`と同じ実装パターン(RUNNINGを返さない。親のFallbackが
 * 2番目の子(向き直り)を毎tick試せるようにするため)。
 *
 * 前回作った`buoy_detector_driver`(BuoyRelativePosition -> bluerov_control_msgs/BuoyDetection
 * への変換アダプタ)経由ではなく、画像処理班の`perception/buoy_detector`が発行する
 * BuoyRelativePositionを直接購読する点に注意(`buoy_detector_driver`とは別経路)。
 *
 * `detected == true` かつ `confidence >= confidence_threshold` ならSUCCESS。
 * それ以外(未受信の場合を含む)はFAILURE(安全側、まだ写っていないとみなす)。
 */
class CheckBuoyInFrame : public BT::ConditionNode
{
public:
  CheckBuoyInFrame(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<buoy_interfaces::msg::BuoyRelativePosition>::SharedPtr detection_sub_;
  buoy_interfaces::msg::BuoyRelativePosition::SharedPtr latest_detection_;

  void detectionCallback(const buoy_interfaces::msg::BuoyRelativePosition::SharedPtr msg);
};

}  // namespace bluerov_control_bt_nodes

#endif
