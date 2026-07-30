#ifndef _CHECK_ROS_BOOL_PARAM_HPP
#define _CHECK_ROS_BOOL_PARAM_HPP

#include <behaviortree_cpp/condition_node.h>

#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief ROSパラメータ(bool)の値でSUCCESS/FAILUREを返す汎用Conditionノード。
 * @details 旧bluerov_controlの debug.image_processing_available /
 * debug.mic_data_from_above という2つのデバッグ用ROSパラメータを、BT側から
 * 同じノード型で参照するために作った(param_nameポートで対象パラメータを切り替える)。
 * RUNNINGは返さない(即座にSUCCESS/FAILUREを返すCondition)。
 */
class CheckRosBoolParam : public BT::ConditionNode
{
public:
  CheckRosBoolParam(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  std::string param_name_;
};

}  // namespace bluerov_control_bt_nodes

#endif
