#ifndef _LOAD_MISSION_PARAMS_HPP
#define _LOAD_MISSION_PARAMS_HPP

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief config/bluerov_mission.param.yaml のミッション定数を ROS パラメータから読み、
 *        BT ブラックボードへ載せる初期化ノード。
 * @details bluerov_phase2.xml 内の {init_delay_ms} 等のプレースホルダが参照する。
 *          ツリー評価の最初に1回だけ SUCCESS を返す(SyncActionNode)。
 */
class LoadMissionParams : public BT::SyncActionNode
{
public:
  LoadMissionParams(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;

  double declare_or_get_double(const std::string & name, double default_value);
};

}  // namespace bluerov_control_bt_nodes

#endif
