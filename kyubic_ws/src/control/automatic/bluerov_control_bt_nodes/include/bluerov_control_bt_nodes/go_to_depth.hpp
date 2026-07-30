#ifndef _GO_TO_DEPTH_HPP
#define _GO_TO_DEPTH_HPP

#include <behaviortree_cpp/action_node.h>

#include <localization_msgs/msg/odometry.hpp>
#include <planner_msgs/msg/wrench_plan.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief 深度目標(絶対値または現在地からの相対オフセット)へ向かうBT葉ノード。
 * @details Phase 1の SetDepthTarget を拡張したもの(旧クラス名からリネーム)。
 *
 * WrenchPlan.targetsはwrench_planner側では自動補完されない(has_master=falseで
 * 自動補完されるのはmaster=現在値のフィードバック側のみ)ため、x/y/roll/yawは
 * このノード自身がodomを購読して起動時点の現在値をtargetsへコピーする
 * (深度以外は現在地を維持したまま深度だけ変える)。
 *
 * ポート:
 *  - depth_absolute_m: 絶対深度目標[m]。指定時はこちらを優先。
 *  - depth_offset_m: 起動時点の深度からの相対オフセット[m](符号込み)。
 *  - どちらも省略した場合はROSパラメータ default_depth_m (bluerov_bt.launch.pyの
 *    depth_target_m引数)へフォールバックする(Phase 1のDESCEND_TO_7M相当との後方互換)。
 *  - tolerance_m: 到達とみなす深度誤差[m](既定0.1)。
 *
 * odomを一度も受信していない間はRUNNINGのまま待機する。目標をpublishした後も
 * 深度がtolerance_m以内に収まるまでRUNNINGを返し続け、収まったらSUCCESSを返す。
 */
class GoToDepth : public BT::StatefulActionNode
{
public:
  GoToDepth(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<planner_msgs::msg::WrenchPlan>::SharedPtr publisher_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr odom_sub_;
  localization_msgs::msg::Odometry::SharedPtr latest_odom_;

  bool published_{false};
  double target_depth_m_{0.0};

  void odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace bluerov_control_bt_nodes

#endif
