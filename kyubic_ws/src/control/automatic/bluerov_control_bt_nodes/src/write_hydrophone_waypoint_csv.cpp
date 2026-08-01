#include "bluerov_control_bt_nodes/write_hydrophone_waypoint_csv.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <planner_msgs/msg/wrench_plan.hpp>
#include <string>

#include "bluerov_control_bt_nodes/waypoint_csv_writer.hpp"

namespace bluerov_control_bt_nodes
{

namespace
{
constexpr double kDegreeToRadian = 3.14159265358979323846 / 180.0;
}  // namespace

WriteHydrophoneWaypointCSV::WriteHydrophoneWaypointCSV(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::StatefulActionNode(name, config), ros_node_(ros_node)
{
  std::string pinger_topic;
  if (!getInput("pinger_direction_topic", pinger_topic)) {
    pinger_topic = "pinger_direction";
  }
  std::string odom_topic_name;
  if (!getInput("odom_topic_name", odom_topic_name)) {
    odom_topic_name = "odom";
  }

  pinger_sub_ = ros_node_->create_subscription<planner_msgs::msg::PingerDirection>(
    pinger_topic, 10,
    std::bind(&WriteHydrophoneWaypointCSV::pingerCallback, this, std::placeholders::_1));
  odom_sub_ = ros_node_->create_subscription<localization_msgs::msg::Odometry>(
    odom_topic_name, 10,
    std::bind(&WriteHydrophoneWaypointCSV::odomCallback, this, std::placeholders::_1));
}

BT::PortsList WriteHydrophoneWaypointCSV::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "pinger_direction_topic", "pinger_direction", "sbl_estimation_nodeが発行するトピック"),
    BT::InputPort<std::string>("odom_topic_name", "odom", "現在姿勢を読むodomトピック"),
    BT::InputPort<double>("step_distance_m", 0.5, "ピンガー方向へ1回で進む距離[m]"),
    BT::InputPort<std::string>(
      "csv_file_path", "書き込むCSVパス('/'始まりでなければpath_plannerのshareディレクトリ基準)")};
}

void WriteHydrophoneWaypointCSV::pingerCallback(
  const planner_msgs::msg::PingerDirection::SharedPtr msg)
{
  latest_pinger_ = msg;
}

void WriteHydrophoneWaypointCSV::odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

BT::NodeStatus WriteHydrophoneWaypointCSV::onStart() { return BT::NodeStatus::RUNNING; }

BT::NodeStatus WriteHydrophoneWaypointCSV::onRunning()
{
  if (!latest_pinger_ || !latest_odom_) {
    // pinger_direction/odomのどちらかをまだ一度も受信していない。安全側に倒し待機する。
    return BT::NodeStatus::RUNNING;
  }

  double step_distance_m = 0.5;
  getInput("step_distance_m", step_distance_m);
  std::string csv_file_path;
  if (!getInput("csv_file_path", csv_file_path) || csv_file_path.empty()) {
    RCLCPP_ERROR(ros_node_->get_logger(), "WriteHydrophoneWaypointCSV: 'csv_file_path' は必須です");
    return BT::NodeStatus::FAILURE;
  }

  // pinger_direction(機体座標系の相対yaw/pitch)を、step_distance_m先の機体座標系
  // オフセットベクトルへ変換する(球面座標->直交座標)。
  const double pinger_yaw_rad = latest_pinger_->yaw * kDegreeToRadian;
  const double pinger_pitch_rad = latest_pinger_->pitch * kDegreeToRadian;
  const double body_offset_x =
    step_distance_m * std::cos(pinger_pitch_rad) * std::cos(pinger_yaw_rad);
  const double body_offset_y =
    step_distance_m * std::cos(pinger_pitch_rad) * std::sin(pinger_yaw_rad);
  // 【未検証・符号反転ポイント】pitch正=上向きという仮定のため、body z(NED慣習で正=下方向)
  // 成分は負にする。実機でpitchの正方向が逆だと判明した場合、この1行の符号だけ反転すればよい。
  const double body_offset_z = -step_distance_m * std::sin(pinger_pitch_rad);

  // 機体座標系のオフセットを、現在姿勢(odom、度単位のroll/pitch/yaw)でNED絶対座標へ回転する
  // (CheckBuoyDetectedと同じtf2::Quaternion::setRPY + quatRotateのパターン)。
  tf2::Quaternion q_rot;
  q_rot.setRPY(
    latest_odom_->pose.orientation.x * kDegreeToRadian,
    latest_odom_->pose.orientation.y * kDegreeToRadian,
    latest_odom_->pose.orientation.z * kDegreeToRadian);
  const tf2::Vector3 body_offset(body_offset_x, body_offset_y, body_offset_z);
  const tf2::Vector3 ned_offset = tf2::quatRotate(q_rot, body_offset);

  const double target_x = latest_odom_->pose.position.x + ned_offset.x();
  const double target_y = latest_odom_->pose.position.y + ned_offset.y();
  const double target_z = latest_odom_->pose.position.z_depth + ned_offset.z();
  // rollは起動時点の現在値を維持する(GoToDepth/GoToBlackboardTargetと同じ設計方針)。
  const double target_roll = latest_odom_->pose.orientation.x;
  // yaw: 通常(step_distance_m>0)は現在値を維持し、位置と深度だけをピンガー方向へ進める
  // (向きは変えない)。ただしstep_distance_m==0.0の場合はその場での「向き直り」用途
  // (iwakuni2026.xmlのTurnTowardPinger)のため、ピンガー方向へ向くようtarget_yawを
  // current_yaw + pinger_yawへ切り替える(x/y/zはoffsetが0になるため現在地のまま)。
  const double target_yaw = (step_distance_m == 0.0)
                              ? latest_odom_->pose.orientation.z + latest_pinger_->yaw
                              : latest_odom_->pose.orientation.z;

  const std::string resolved_path =
    waypoint_csv_writer::ResolvePath(ros_node_->get_logger(), csv_file_path);
  try {
    waypoint_csv_writer::WriteSingleCheckpoint(
      resolved_path, target_x, target_y, target_z, planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH,
      target_roll, target_yaw);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "WriteHydrophoneWaypointCSV: CSV書き込みに失敗しました: %s",
      e.what());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "WriteHydrophoneWaypointCSV: target=(%.2f, %.2f, %.2f) を %s へ書き込みました", target_x,
    target_y, target_z, resolved_path.c_str());
  return BT::NodeStatus::SUCCESS;
}

void WriteHydrophoneWaypointCSV::onHalted() {}

}  // namespace bluerov_control_bt_nodes
