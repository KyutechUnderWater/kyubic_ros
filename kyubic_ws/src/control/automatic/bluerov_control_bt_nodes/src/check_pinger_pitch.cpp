#include "bluerov_control_bt_nodes/check_pinger_pitch.hpp"

namespace bluerov_control_bt_nodes
{

CheckPingerPitch::CheckPingerPitch(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::ConditionNode(name, config), ros_node_(ros_node)
{
  std::string pinger_topic;
  if (!getInput("pinger_direction_topic", pinger_topic)) {
    pinger_topic = "pinger_direction";
  }

  pinger_sub_ = ros_node_->create_subscription<planner_msgs::msg::PingerDirection>(
    pinger_topic, 10, std::bind(&CheckPingerPitch::pingerCallback, this, std::placeholders::_1));
}

BT::PortsList CheckPingerPitch::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "pinger_direction_topic", "pinger_direction", "sbl_controlが発行するトピック(相対名。launchでremap)"),
    BT::InputPort<double>(
      "pitch_threshold_deg", 50.0, "これ以下なら十分近いとみなす(未検証、実機で符号・向きを検証)")};
}

void CheckPingerPitch::pingerCallback(const planner_msgs::msg::PingerDirection::SharedPtr msg)
{
  latest_pinger_ = msg;
}

BT::NodeStatus CheckPingerPitch::tick()
{
  if (!latest_pinger_) {
    return BT::NodeStatus::FAILURE;
  }

  double pitch_threshold_deg = 50.0;
  getInput("pitch_threshold_deg", pitch_threshold_deg);

  return latest_pinger_->pitch <= pitch_threshold_deg ? BT::NodeStatus::SUCCESS
                                                      : BT::NodeStatus::FAILURE;
}

}  // namespace bluerov_control_bt_nodes
