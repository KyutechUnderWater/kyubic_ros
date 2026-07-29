#ifndef _SET_DEPTH_TARGET_HPP
#define _SET_DEPTH_TARGET_HPP

#include <behaviortree_cpp/action_node.h>

#include <localization_msgs/msg/odometry.hpp>
#include <planner_msgs/msg/wrench_plan.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief 絶対深度目標を1回だけ zoh_wrench_plan へpublishするBT葉ノード。
 * @details WrenchPlan.targetsはwrench_planner側では自動補完されない(has_master=falseで
 * 自動補完されるのはmaster=現在値のフィードバック側のみ)ため、x/y/roll/yawは
 * このノード自身がodomを購読して現在値をtargetsへコピーする(「今いる場所からdepthだけ
 * 絶対値で目標にする」= 現在地のx,y,roll,yawを維持したまま深度だけ変える、という
 * 旧bluerov_control/flow.pyのDESCEND_TO_7M相当の挙動)。
 * odomを一度も受信していない間はRUNNINGのまま待機し、受信後に1回だけpublishしてSUCCESSを返す。
 */
class SetDepthTarget : public BT::StatefulActionNode
{
public:
  SetDepthTarget(
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

  void odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace bluerov_control_bt_nodes

#endif
