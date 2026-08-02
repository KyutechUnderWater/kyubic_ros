#include "bluerov_control_bt_nodes/write_vision_waypoint_csv.hpp"

#include <cmath>
#include <planner_msgs/msg/wrench_plan.hpp>

#include "bluerov_control_bt_nodes/buoy_detection_csv_reader.hpp"
#include "bluerov_control_bt_nodes/waypoint_csv_writer.hpp"

namespace bluerov_control_bt_nodes
{

namespace
{
constexpr double kDegreeToRadian = 3.14159265358979323846 / 180.0;
constexpr double kRadianToDegree = 180.0 / 3.14159265358979323846;
}  // namespace

WriteVisionWaypointCSV::WriteVisionWaypointCSV(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::StatefulActionNode(name, config), ros_node_(ros_node)
{
  std::string odom_topic_name;
  if (!getInput("odom_topic_name", odom_topic_name)) {
    odom_topic_name = "odom";
  }

  odom_sub_ = ros_node_->create_subscription<localization_msgs::msg::Odometry>(
    odom_topic_name, 10,
    std::bind(&WriteVisionWaypointCSV::odomCallback, this, std::placeholders::_1));
}

BT::PortsList WriteVisionWaypointCSV::providedPorts()
{
  return {
    BT::InputPort<std::string>("odom_topic_name", "odom", "現在姿勢を読むodomトピック"),
    BT::InputPort<double>("step_distance_m", 0.3, "1回で前進する距離[m](暫定値)"),
    BT::InputPort<double>(
      "vertical_gain_m", 0.2, "垂直補正の比例ゲイン(無次元。CSVのz値に乗じる。暫定値)"),
    BT::InputPort<double>("deadband_rad", 0.035, "これ未満のyaw補正は0にする不感帯[rad](暫定値)"),
    BT::InputPort<double>(
      "horizontal_fov_rad", "カメラの水平画角[rad]。yaw_valid=falseの場合のみ使用"),
    BT::InputPort<double>("confidence_threshold", 0.5, "これ未満の検出は無視する(暫定値)"),
    BT::InputPort<double>(
      "stale_timeout_sec", 1.0,
      "buoy_detector_node.pyのCSV最終更新時刻がこれより古い場合は無視する[s]"),
    BT::InputPort<std::string>(
      "csv_file_path", "書き込むCSVパス('/'始まりでなければpath_plannerのshareディレクトリ基準)")};
}

void WriteVisionWaypointCSV::odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

BT::NodeStatus WriteVisionWaypointCSV::onStart() { return BT::NodeStatus::RUNNING; }

BT::NodeStatus WriteVisionWaypointCSV::onRunning()
{
  if (!latest_odom_) {
    // odomをまだ一度も受信していない。安全側に倒し待機する。
    return BT::NodeStatus::RUNNING;
  }

  double stale_timeout_sec = 1.0;
  getInput("stale_timeout_sec", stale_timeout_sec);

  buoy_detection_csv_reader::BuoyDetectionRow row;
  if (!buoy_detection_csv_reader::ReadLatest(
        buoy_detection_csv_reader::kBuoyDetectionCsvPath, stale_timeout_sec, row)) {
    // CSV未生成、更新が古い(stale_timeout_sec超過)、または最終行を読めない。安全側に待機する。
    return BT::NodeStatus::RUNNING;
  }

  double confidence_threshold = 0.5;
  getInput("confidence_threshold", confidence_threshold);

  // ①confidence不足、または未検出ならこのtickは何もしない(FAILURE、次tickで再試行)。
  if (!row.detected || row.confidence < confidence_threshold) {
    return BT::NodeStatus::FAILURE;
  }

  // ②yaw補正: yaw_validならyaw_radをそのまま使う。そうでなければ
  // normalized_horizontal_offset × (horizontal_fov_rad / 2)で近似する。
  double yaw_correction_rad = 0.0;
  if (row.yaw_valid) {
    yaw_correction_rad = row.yaw_rad;
  } else {
    double horizontal_fov_rad = 0.0;
    if (!getInput("horizontal_fov_rad", horizontal_fov_rad)) {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "WriteVisionWaypointCSV: yaw_valid=falseかつ'horizontal_fov_rad'が未指定のため"
        "yaw補正を計算できません");
      return BT::NodeStatus::FAILURE;
    }
    yaw_correction_rad = row.normalized_horizontal_offset * (horizontal_fov_rad / 2.0);
  }

  // ③デッドバンド(ジッター防止)。
  double deadband_rad = 0.035;
  getInput("deadband_rad", deadband_rad);
  if (std::abs(yaw_correction_rad) < deadband_rad) {
    yaw_correction_rad = 0.0;
  }

  // ④新しいyaw[rad](odomのyawは度単位のため変換する)。
  const double current_yaw_rad = latest_odom_->pose.orientation.z * kDegreeToRadian;
  const double new_yaw_rad = current_yaw_rad + yaw_correction_rad;

  // ⑤z_step。【未検証・要実機確認】CSVのz値は機体座標系で正=下方向(NED慣習と
  // 一致する前提)。vertical_gain_mは無次元の比例ゲインとして扱う。
  double vertical_gain_m = 0.2;
  getInput("vertical_gain_m", vertical_gain_m);
  const double z_step = row.position_valid ? row.z_m * vertical_gain_m : 0.0;

  // ⑥target_x/y/z/yaw。rollは常に現在値を維持。
  double step_distance_m = 0.3;
  getInput("step_distance_m", step_distance_m);
  const double target_x = latest_odom_->pose.position.x + step_distance_m * std::cos(new_yaw_rad);
  const double target_y = latest_odom_->pose.position.y + step_distance_m * std::sin(new_yaw_rad);
  const double target_z = latest_odom_->pose.position.z_depth + z_step;
  const double target_yaw = new_yaw_rad * kRadianToDegree;
  const double target_roll = latest_odom_->pose.orientation.x;

  std::string csv_file_path;
  if (!getInput("csv_file_path", csv_file_path) || csv_file_path.empty()) {
    RCLCPP_ERROR(ros_node_->get_logger(), "WriteVisionWaypointCSV: 'csv_file_path' は必須です");
    return BT::NodeStatus::FAILURE;
  }

  const std::string resolved_path =
    waypoint_csv_writer::ResolvePath(ros_node_->get_logger(), csv_file_path);
  try {
    waypoint_csv_writer::WriteSingleCheckpoint(
      resolved_path, target_x, target_y, target_z, planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH,
      target_roll, target_yaw);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "WriteVisionWaypointCSV: CSV書き込みに失敗しました: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "WriteVisionWaypointCSV: target=(%.2f, %.2f, %.2f, yaw=%.1f) を %s へ書き込みました", target_x,
    target_y, target_z, target_yaw, resolved_path.c_str());
  return BT::NodeStatus::SUCCESS;
}

void WriteVisionWaypointCSV::onHalted() {}

}  // namespace bluerov_control_bt_nodes
