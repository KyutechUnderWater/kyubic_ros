#ifndef _CHECK_PINGER_FOUND_HPP
#define _CHECK_PINGER_FOUND_HPP

#include <behaviortree_cpp/action_node.h>

#include <planner_msgs/msg/pinger_direction.hpp>
#include <planner_msgs/msg/wrench_plan.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief ハイドロフォンでピンガーを発見したかを判定するBT葉ノード(旧flow.pyの
 * handle_search_hydrophoneの発見判定部分)。
 * @details 移動そのものはこのノードの責務ではない。sbl_controller_node(音響班、外部)が
 * pinger_directionから決定した目標を継続的にwrench_plan_topic(既定zoh_wrench_plan)へ
 * publishし続け、ZeroOrderHold経由でwrench_plannerがそれに追従し続ける(このノードは
 * 何もpublishしない、判定専任)。
 *
 * pinger_direction.pitchがpitch_threshold_degを超え、かつscoreが
 * score_threshold以下(scoreは最適化誤差なので小さいほど良い)になった瞬間、
 * その時点でwrench_plan_topicが示していた絶対目標をtarget_x/y/zポートへ出力してSUCCESS。
 * pinger_direction/wrench_planのどちらか未受信、または閾値未達の間はRUNNING
 * (安全側で待機を継続。誤ってSUCCESSを返さない)。
 */
class CheckPingerFound : public BT::StatefulActionNode
{
public:
  CheckPingerFound(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<planner_msgs::msg::PingerDirection>::SharedPtr pinger_sub_;
  rclcpp::Subscription<planner_msgs::msg::WrenchPlan>::SharedPtr wrench_plan_sub_;
  planner_msgs::msg::PingerDirection::SharedPtr latest_pinger_;
  planner_msgs::msg::WrenchPlan::SharedPtr latest_wrench_plan_;

  void pingerCallback(const planner_msgs::msg::PingerDirection::SharedPtr msg);
  void wrenchPlanCallback(const planner_msgs::msg::WrenchPlan::SharedPtr msg);
};

}  // namespace bluerov_control_bt_nodes

#endif
