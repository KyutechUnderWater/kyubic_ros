#include "bluerov_control_bt_nodes/check_buoy_position.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "bluerov_control_bt_nodes/buoy_detection_csv_reader.hpp"

namespace bluerov_control_bt_nodes
{

namespace
{
constexpr double kDegreeToRadian = 3.14159265358979323846 / 180.0;
}  // namespace

CheckBuoyPosition::CheckBuoyPosition(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::StatefulActionNode(name, config), ros_node_(ros_node)
{
  std::string odom_topic_name;
  if (!getInput("odom_topic_name", odom_topic_name)) {
    odom_topic_name = "odom";
  }

  odom_sub_ = ros_node_->create_subscription<localization_msgs::msg::Odometry>(
    odom_topic_name, 10, std::bind(&CheckBuoyPosition::odomCallback, this, std::placeholders::_1));
}

BT::PortsList CheckBuoyPosition::providedPorts()
{
  return {
    BT::InputPort<std::string>("odom_topic_name", "odom", "現在姿勢を読むodomトピック"),
    BT::InputPort<double>("confidence_threshold", 0.5, "これ未満の検出は無視する"),
    BT::InputPort<double>(
      "stale_timeout_sec", 1.0,
      "buoy_detector_node.pyのCSV最終更新時刻がこれより古い場合は無視する[s]"),
    BT::OutputPort<double>("target_x", "検出したブイの絶対x座標[m](ブラックボードへ出力)"),
    BT::OutputPort<double>("target_y", "検出したブイの絶対y座標[m](ブラックボードへ出力)"),
    BT::OutputPort<double>("target_z", "検出したブイの絶対深度z_depth[m](ブラックボードへ出力)")};
}

void CheckBuoyPosition::odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

BT::NodeStatus CheckBuoyPosition::onStart() { return BT::NodeStatus::RUNNING; }

BT::NodeStatus CheckBuoyPosition::onRunning()
{
  if (!latest_odom_) {
    // odomをまだ一度も受信していない。安全側に倒し待機する。
    return BT::NodeStatus::RUNNING;
  }

  double confidence_threshold = 0.5;
  double stale_timeout_sec = 1.0;
  getInput("confidence_threshold", confidence_threshold);
  getInput("stale_timeout_sec", stale_timeout_sec);

  buoy_detection_csv_reader::BuoyDetectionRow row;
  if (!buoy_detection_csv_reader::ReadLatest(
        buoy_detection_csv_reader::kBuoyDetectionCsvPath, stale_timeout_sec, row)) {
    // CSV未生成、更新が古い(stale_timeout_sec超過)、または最終行を読めない。安全側に待機する。
    return BT::NodeStatus::RUNNING;
  }

  if (!row.detected || !row.position_valid || row.confidence < confidence_threshold) {
    // 未検出、位置無効、またはconfidence不足。安全側に待機する。
    return BT::NodeStatus::RUNNING;
  }

  // 機体座標系の相対オフセットをNED絶対座標へ回転する(CheckBuoyDetectedと同じ
  // tf2::Quaternion::setRPYベースの変換)。
  tf2::Quaternion q_rot;
  q_rot.setRPY(
    latest_odom_->pose.orientation.x * kDegreeToRadian,
    latest_odom_->pose.orientation.y * kDegreeToRadian,
    latest_odom_->pose.orientation.z * kDegreeToRadian);
  const tf2::Vector3 body_offset(row.x_m, row.y_m, row.z_m);
  const tf2::Vector3 ned_offset = tf2::quatRotate(q_rot, body_offset);

  const double target_x = latest_odom_->pose.position.x + ned_offset.x();
  const double target_y = latest_odom_->pose.position.y + ned_offset.y();
  const double target_z = latest_odom_->pose.position.z_depth + ned_offset.z();
  setOutput("target_x", target_x);
  setOutput("target_y", target_y);
  setOutput("target_z", target_z);
  RCLCPP_INFO(
    ros_node_->get_logger(), "CheckBuoyPosition: 検出(confidence=%.2f) target=(%.2f, %.2f, %.2f)",
    row.confidence, target_x, target_y, target_z);
  return BT::NodeStatus::SUCCESS;
}

void CheckBuoyPosition::onHalted() {}

}  // namespace bluerov_control_bt_nodes
