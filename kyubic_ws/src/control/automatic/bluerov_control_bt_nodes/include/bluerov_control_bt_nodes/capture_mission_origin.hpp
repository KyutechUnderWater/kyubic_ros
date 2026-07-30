#ifndef _CAPTURE_MISSION_ORIGIN_HPP
#define _CAPTURE_MISSION_ORIGIN_HPP

#include <behaviortree_cpp/action_node.h>

#include <localization_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief フィールド境界判定(GoToBlackboardTarget)の原点を確定するBT葉ノード。
 * @details odom(相対トピック、既定"odom")を購読し、status.dvl.id ==
 * common_msgs::msg::Status::NORMAL のodomを受信するまでRUNNINGのまま待機する
 * (単に最初の1通ではない)。ミッション開始直後はDVLがまだ水底ロックできておらず、
 * 最初のodomがDVL ERROR(x/y=held/未確定な値の可能性がある)である場合があるため、
 * Part Aの has_valid_measurement_ と同じ考え方でstatus.dvl.idを明示的に確認してから
 * mission_origin_x/yをブラックボードへ確定する。
 *
 * このノード自体は個別のタイムアウトを持たない(GoToDepth等と同じ設計)。DVLが
 * 一度もNORMALにならない場合(壁際等)に無期限RUNNINGし続けないよう、XML側で
 * <Timeout msec="{origin_capture_timeout_ms}"><CaptureMissionOrigin/></Timeout>
 * のように明示的に包むこと。
 */
class CaptureMissionOrigin : public BT::StatefulActionNode
{
public:
  CaptureMissionOrigin(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr odom_sub_;
  localization_msgs::msg::Odometry::SharedPtr latest_normal_odom_;

  void odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace bluerov_control_bt_nodes

#endif
