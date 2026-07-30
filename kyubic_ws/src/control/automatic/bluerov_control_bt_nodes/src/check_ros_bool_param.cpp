#include "bluerov_control_bt_nodes/check_ros_bool_param.hpp"

namespace bluerov_control_bt_nodes
{

CheckRosBoolParam::CheckRosBoolParam(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::ConditionNode(name, config), ros_node_(ros_node)
{
  if (!getInput("param_name", param_name_)) {
    throw BT::RuntimeError("CheckRosBoolParam: 'param_name' ポートは必須です");
  }

  bool default_value = true;
  getInput("default_value", default_value);
  if (!ros_node_->has_parameter(param_name_)) {
    ros_node_->declare_parameter<bool>(param_name_, default_value);
  }
}

BT::PortsList CheckRosBoolParam::providedPorts()
{
  return {
    BT::InputPort<std::string>("param_name", "参照するROSパラメータ名(例: debug.mic_data_from_above)"),
    BT::InputPort<bool>("default_value", true, "パラメータが未宣言の場合の既定値")};
}

BT::NodeStatus CheckRosBoolParam::tick()
{
  const bool value = ros_node_->get_parameter(param_name_).as_bool();
  return value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace bluerov_control_bt_nodes
