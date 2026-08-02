#ifndef _CHECK_BUOY_IN_FRAME_HPP
#define _CHECK_BUOY_IN_FRAME_HPP

#include <behaviortree_cpp/condition_node.h>

#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief perception/buoy_detector(buoy_detector_node.py)が書き出す固定CSVの最終行を
 * (buoy_detection_csv_reader経由で)読み、ブイがカメラに写っているかを毎tick即座に判定する
 * Conditionノード(iwakuni2026.xml、`Final Descend Successful`直後の「写っていなければ
 * ハイドロフォン方向へ向き直る」ループ)。
 * @details `CheckPingerPitch`と同じ実装パターン(RUNNINGを返さない。親のFallbackが
 * 2番目の子(向き直り)を毎tick試せるようにするため)。
 *
 * 以前は`/buoy/relative_position`(buoy_interfaces/msg/BuoyRelativePosition)を直接購読して
 * いたが、buoy_detector_node.pyが同じ検出値を毎フレーム追記している固定CSVを
 * `buoy_detection_csv_reader`経由で読み取る方式に変更した(パス・列レイアウトは
 * buoy_detection_csv_reader.hpp参照)。
 *
 * `detected == true` かつ `confidence >= confidence_threshold` ならSUCCESS。
 * それ以外(CSV未生成・stale_timeout_sec超過・パース不能の場合を含む)はFAILURE(安全側、
 * まだ写っていないとみなす)。
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
};

}  // namespace bluerov_control_bt_nodes

#endif
