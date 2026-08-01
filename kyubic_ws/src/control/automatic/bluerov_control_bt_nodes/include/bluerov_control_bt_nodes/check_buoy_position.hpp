#ifndef _CHECK_BUOY_POSITION_HPP
#define _CHECK_BUOY_POSITION_HPP

#include <behaviortree_cpp/action_node.h>

#include <buoy_interfaces/msg/buoy_relative_position.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief 画像処理班の生データ(buoy_interfaces/msg/BuoyRelativePosition)を直接購読し、
 * 機体座標系の相対位置をNED絶対座標へ変換するBT葉ノード(iwakuni2026.xml、FinalApproach用)。
 * @details `CheckBuoyDetected`(§4、`bluerov_control_msgs/msg/BuoyDetection`を
 * `buoy_detector_driver`経由で購読)とは別経路。`CheckBuoyInFrame`/`WriteVisionWaypointCSV`と
 * 同じく`/buoy/relative_position`を直接見るため、`buoy_detector_driver`の起動を必要としない。
 *
 * `detected==false`、`position_valid==false`(x/y/zがNaN)、confidenceが閾値未満、または
 * timestampがstale_timeout_sec超過の検出は使わない。有効な検出があれば、odomの姿勢を使い
 * 機体座標系の相対オフセット(relative_buoy_x/y/z_m)をNED絶対座標へ回転させ(CheckBuoyDetected
 * と同じtf2::Quaternion::setRPY + quatRotateのパターン)、target_x/y/zポートへ出力してSUCCESS。
 * それ以外はRUNNING(安全側で待機、誤ってSUCCESSを返さない)。
 */
class CheckBuoyPosition : public BT::StatefulActionNode
{
public:
  CheckBuoyPosition(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<buoy_interfaces::msg::BuoyRelativePosition>::SharedPtr detection_sub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr odom_sub_;
  buoy_interfaces::msg::BuoyRelativePosition::SharedPtr latest_detection_;
  localization_msgs::msg::Odometry::SharedPtr latest_odom_;

  void detectionCallback(const buoy_interfaces::msg::BuoyRelativePosition::SharedPtr msg);
  void odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace bluerov_control_bt_nodes

#endif
