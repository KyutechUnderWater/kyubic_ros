#ifndef _CHECK_BUOY_DETECTED_HPP
#define _CHECK_BUOY_DETECTED_HPP

#include <behaviortree_cpp/action_node.h>

#include <bluerov_control_msgs/msg/buoy_detection.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief 画像処理によるブイ検出を判定し、機体座標系の相対位置をNED絶対座標へ変換するBT葉ノード
 * (旧flow.pyのhandle_visual_detect、perception.is_buoy_detection_usable/body_offset_to_nedに相当)。
 * @details detected==false、confidenceが閾値未満、またはtimestampがstale_timeout_sec超過の
 * 検出は使わない(旧is_buoy_detection_usableと同じ判定)。有効な検出があれば、odomの姿勢を使い
 * 機体座標系の相対オフセット(relative_buoy_x/y/z_m)をNED絶対座標へ回転させ、target_x/y/zポートへ
 * 出力してSUCCESS。それ以外はRUNNING(安全側で待機、誤ってSUCCESSを返さない)。
 */
class CheckBuoyDetected : public BT::StatefulActionNode
{
public:
  CheckBuoyDetected(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<bluerov_control_msgs::msg::BuoyDetection>::SharedPtr detection_sub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr odom_sub_;
  bluerov_control_msgs::msg::BuoyDetection::SharedPtr latest_detection_;
  localization_msgs::msg::Odometry::SharedPtr latest_odom_;

  void detectionCallback(const bluerov_control_msgs::msg::BuoyDetection::SharedPtr msg);
  void odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace bluerov_control_bt_nodes

#endif
