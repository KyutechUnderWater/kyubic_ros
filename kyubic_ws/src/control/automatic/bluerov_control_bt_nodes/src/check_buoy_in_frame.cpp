#include "bluerov_control_bt_nodes/check_buoy_in_frame.hpp"

namespace bluerov_control_bt_nodes
{

CheckBuoyInFrame::CheckBuoyInFrame(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::ConditionNode(name, config), ros_node_(ros_node)
{
  std::string topic_name;
  if (!getInput("buoy_relative_position_topic", topic_name)) {
    topic_name = "/buoy/relative_position";
  }

  detection_sub_ = ros_node_->create_subscription<buoy_interfaces::msg::BuoyRelativePosition>(
    topic_name, 10, std::bind(&CheckBuoyInFrame::detectionCallback, this, std::placeholders::_1));
}

BT::PortsList CheckBuoyInFrame::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "buoy_relative_position_topic", "/buoy/relative_position",
      "perception/buoy_detectorが発行するトピック(buoy_detector_driver経由ではない生データ)"),
    BT::InputPort<double>(
      "confidence_threshold", 0.5,
      "これ以上ならブイが写っているとみなす(暫定値。実測データで要調整)")};
}

void CheckBuoyInFrame::detectionCallback(
  const buoy_interfaces::msg::BuoyRelativePosition::SharedPtr msg)
{
  latest_detection_ = msg;
}

BT::NodeStatus CheckBuoyInFrame::tick()
{
  if (!latest_detection_) {
    // buoy_relative_positionを一度も受信していない。安全側に倒し、まだ写っていないとみなす。
    return BT::NodeStatus::FAILURE;
  }

  double confidence_threshold = 0.5;
  getInput("confidence_threshold", confidence_threshold);

  return (latest_detection_->detected && latest_detection_->confidence >= confidence_threshold)
           ? BT::NodeStatus::SUCCESS
           : BT::NodeStatus::FAILURE;
}

}  // namespace bluerov_control_bt_nodes
