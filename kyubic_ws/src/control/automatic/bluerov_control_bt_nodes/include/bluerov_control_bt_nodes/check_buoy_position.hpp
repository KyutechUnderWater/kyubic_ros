#ifndef _CHECK_BUOY_POSITION_HPP
#define _CHECK_BUOY_POSITION_HPP

#include <behaviortree_cpp/action_node.h>

#include <localization_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief perception/buoy_detector(buoy_detector_node.py)が書き出す固定CSVの最終行を読み、
 * 機体座標系の相対位置をNED絶対座標へ変換するBT葉ノード(iwakuni2026.xml、FinalApproach用)。
 * @details 以前は`/buoy/relative_position`(buoy_interfaces/msg/BuoyRelativePosition)を
 * 直接購読していたが、buoy_detector_node.pyが同じ検出値を毎フレーム追記している固定CSVを
 * `buoy_detection_csv_reader`経由で読み取る方式に変更した(パス・列レイアウトは
 * buoy_detection_csv_reader.hpp参照)。
 *
 * detected==false、x/y/zが空欄(位置無効)、confidenceが閾値未満、またはCSVの最終更新時刻が
 * stale_timeout_sec超過の場合は使わない。有効な検出があれば、odomの姿勢を使い機体座標系の
 * 相対オフセットをNED絶対座標へ回転させ(CheckBuoyDetectedと同じtf2::Quaternion::setRPY +
 * quatRotateのパターン)、target_x/y/zポートへ出力してSUCCESS。それ以外はRUNNING(安全側で
 * 待機、誤ってSUCCESSを返さない)。
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
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr odom_sub_;
  localization_msgs::msg::Odometry::SharedPtr latest_odom_;

  void odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace bluerov_control_bt_nodes

#endif
