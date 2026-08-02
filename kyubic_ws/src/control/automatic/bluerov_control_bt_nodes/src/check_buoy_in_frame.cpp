#include "bluerov_control_bt_nodes/check_buoy_in_frame.hpp"

#include "bluerov_control_bt_nodes/buoy_detection_csv_reader.hpp"

namespace bluerov_control_bt_nodes
{

CheckBuoyInFrame::CheckBuoyInFrame(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::ConditionNode(name, config), ros_node_(ros_node)
{
}

BT::PortsList CheckBuoyInFrame::providedPorts()
{
  return {
    BT::InputPort<double>(
      "confidence_threshold", 0.5,
      "これ以上ならブイが写っているとみなす(暫定値。実測データで要調整)"),
    BT::InputPort<double>(
      "stale_timeout_sec", 1.0,
      "buoy_detector_node.pyのCSV最終更新時刻がこれより古い場合は無視する[s]")};
}

BT::NodeStatus CheckBuoyInFrame::tick()
{
  double stale_timeout_sec = 1.0;
  getInput("stale_timeout_sec", stale_timeout_sec);

  buoy_detection_csv_reader::BuoyDetectionRow row;
  if (!buoy_detection_csv_reader::ReadLatest(
        buoy_detection_csv_reader::kBuoyDetectionCsvPath, stale_timeout_sec, row)) {
    // CSV未生成、更新が古い(stale_timeout_sec超過)、または最終行を読めない。安全側に倒し、
    // まだ写っていないとみなす。
    return BT::NodeStatus::FAILURE;
  }

  double confidence_threshold = 0.5;
  getInput("confidence_threshold", confidence_threshold);

  return (row.detected && row.confidence >= confidence_threshold) ? BT::NodeStatus::SUCCESS
                                                                    : BT::NodeStatus::FAILURE;
}

}  // namespace bluerov_control_bt_nodes
