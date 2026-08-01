#include "bluerov_control_bt_nodes/check_buoy_position.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

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
  std::string detection_topic;
  if (!getInput("buoy_relative_position_topic", detection_topic)) {
    detection_topic = "/buoy/relative_position";
  }
  std::string odom_topic_name;
  if (!getInput("odom_topic_name", odom_topic_name)) {
    odom_topic_name = "odom";
  }

  detection_sub_ = ros_node_->create_subscription<buoy_interfaces::msg::BuoyRelativePosition>(
    detection_topic, 10,
    std::bind(&CheckBuoyPosition::detectionCallback, this, std::placeholders::_1));
  odom_sub_ = ros_node_->create_subscription<localization_msgs::msg::Odometry>(
    odom_topic_name, 10, std::bind(&CheckBuoyPosition::odomCallback, this, std::placeholders::_1));
}

BT::PortsList CheckBuoyPosition::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "buoy_relative_position_topic", "/buoy/relative_position",
      "perception/buoy_detectorが発行するトピック(buoy_detector_driver経由ではない生データ)"),
    BT::InputPort<std::string>("odom_topic_name", "odom", "現在姿勢を読むodomトピック"),
    BT::InputPort<double>("confidence_threshold", 0.5, "これ未満の検出は無視する"),
    BT::InputPort<double>("stale_timeout_sec", 1.0, "timestampがこれより古い検出は無視する[s]"),
    BT::OutputPort<double>("target_x", "検出したブイの絶対x座標[m](ブラックボードへ出力)"),
    BT::OutputPort<double>("target_y", "検出したブイの絶対y座標[m](ブラックボードへ出力)"),
    BT::OutputPort<double>("target_z", "検出したブイの絶対深度z[m](ブラックボードへ出力)")};
}

void CheckBuoyPosition::detectionCallback(
  const buoy_interfaces::msg::BuoyRelativePosition::SharedPtr msg)
{
  latest_detection_ = msg;
}

void CheckBuoyPosition::odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

BT::NodeStatus CheckBuoyPosition::onStart() { return BT::NodeStatus::RUNNING; }

BT::NodeStatus CheckBuoyPosition::onRunning()
{
  if (!latest_detection_ || !latest_odom_) {
    // buoy_relative_position/odomのどちらかをまだ一度も受信していない。安全側に倒し待機する。
    return BT::NodeStatus::RUNNING;
  }

  double confidence_threshold = 0.5;
  double stale_timeout_sec = 1.0;
  getInput("confidence_threshold", confidence_threshold);
  getInput("stale_timeout_sec", stale_timeout_sec);

  if (
    !latest_detection_->detected || !latest_detection_->position_valid ||
    latest_detection_->confidence < confidence_threshold) {
    // 未検出、x/y/zがNaN(position_valid=false)、またはconfidence不足。安全側に待機する。
    return BT::NodeStatus::RUNNING;
  }
  const rclcpp::Time stamp(latest_detection_->header.stamp);
  const double age_sec = (ros_node_->get_clock()->now() - stamp).seconds();
  if (age_sec > stale_timeout_sec) {
    return BT::NodeStatus::RUNNING;
  }

  // 機体座標系の相対オフセットをNED絶対座標へ回転する(CheckBuoyDetectedと同じ
  // tf2::Quaternion::setRPYベースの変換)。
  tf2::Quaternion q_rot;
  q_rot.setRPY(
    latest_odom_->pose.orientation.x * kDegreeToRadian,
    latest_odom_->pose.orientation.y * kDegreeToRadian,
    latest_odom_->pose.orientation.z * kDegreeToRadian);
  const tf2::Vector3 body_offset(
    latest_detection_->relative_buoy_x_m, latest_detection_->relative_buoy_y_m,
    latest_detection_->relative_buoy_z_m);
  const tf2::Vector3 ned_offset = tf2::quatRotate(q_rot, body_offset);

  const double target_x = latest_odom_->pose.position.x + ned_offset.x();
  const double target_y = latest_odom_->pose.position.y + ned_offset.y();
  const double target_z = latest_odom_->pose.position.z_depth + ned_offset.z();
  setOutput("target_x", target_x);
  setOutput("target_y", target_y);
  setOutput("target_z", target_z);
  RCLCPP_INFO(
    ros_node_->get_logger(), "CheckBuoyPosition: 検出(confidence=%.2f) target=(%.2f, %.2f, %.2f)",
    latest_detection_->confidence, target_x, target_y, target_z);
  return BT::NodeStatus::SUCCESS;
}

void CheckBuoyPosition::onHalted() {}

}  // namespace bluerov_control_bt_nodes
