#include "bluerov_control_bt_nodes/write_axis_override_waypoint_csv.hpp"

#include <planner_msgs/msg/wrench_plan.hpp>

#include "bluerov_control_bt_nodes/waypoint_csv_writer.hpp"

namespace bluerov_control_bt_nodes
{

WriteAxisOverrideWaypointCSV::WriteAxisOverrideWaypointCSV(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::StatefulActionNode(name, config), ros_node_(ros_node)
{
  std::string odom_topic_name;
  if (!getInput("odom_topic_name", odom_topic_name)) {
    odom_topic_name = "odom";
  }

  odom_sub_ = ros_node_->create_subscription<localization_msgs::msg::Odometry>(
    odom_topic_name, 10,
    std::bind(&WriteAxisOverrideWaypointCSV::odomCallback, this, std::placeholders::_1));
}

BT::PortsList WriteAxisOverrideWaypointCSV::providedPorts()
{
  return {
    BT::InputPort<std::string>("odom_topic_name", "odom", "現在姿勢を読むodomトピック"),
    BT::InputPort<double>("override_x", "絶対x座標[m]。省略時は現在値を維持"),
    BT::InputPort<double>("override_y", "絶対y座標[m]。省略時は現在値を維持"),
    BT::InputPort<double>("override_z", "絶対z座標[m](z_modeの基準に従う)。省略時は現在値を維持"),
    BT::InputPort<double>("override_yaw", "絶対yaw[deg]。省略時は現在値を維持"),
    BT::InputPort<std::string>(
      "z_mode", "depth", "\"depth\"(z_depth基準)または\"altitude\"(z_altitude基準)"),
    BT::InputPort<std::string>(
      "csv_file_path", "書き込むCSVパス('/'始まりでなければpath_plannerのshareディレクトリ基準)")};
}

void WriteAxisOverrideWaypointCSV::odomCallback(
  const localization_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

BT::NodeStatus WriteAxisOverrideWaypointCSV::onStart() { return BT::NodeStatus::RUNNING; }

BT::NodeStatus WriteAxisOverrideWaypointCSV::onRunning()
{
  if (!latest_odom_) {
    // odomをまだ一度も受信していない。安全側に倒し、届くまで待機する。
    return BT::NodeStatus::RUNNING;
  }

  std::string csv_file_path;
  if (!getInput("csv_file_path", csv_file_path) || csv_file_path.empty()) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "WriteAxisOverrideWaypointCSV: 'csv_file_path' は必須です");
    return BT::NodeStatus::FAILURE;
  }

  std::string z_mode_str = "depth";
  getInput("z_mode", z_mode_str);
  const bool is_altitude = (z_mode_str == "altitude");
  const uint8_t z_mode = is_altitude ? planner_msgs::msg::WrenchPlan::Z_MODE_ALTITUDE
                                     : planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH;
  const double current_z =
    is_altitude ? latest_odom_->pose.position.z_altitude : latest_odom_->pose.position.z_depth;

  double override_x = 0.0;
  double override_y = 0.0;
  double override_z = 0.0;
  double override_yaw = 0.0;
  const double target_x =
    getInput("override_x", override_x) ? override_x : latest_odom_->pose.position.x;
  const double target_y =
    getInput("override_y", override_y) ? override_y : latest_odom_->pose.position.y;
  const double target_z = getInput("override_z", override_z) ? override_z : current_z;
  const double target_yaw =
    getInput("override_yaw", override_yaw) ? override_yaw : latest_odom_->pose.orientation.z;
  // rollはこのノードでは上書き対象にしない。常に現在値を使う。
  const double target_roll = latest_odom_->pose.orientation.x;

  const std::string resolved_path =
    waypoint_csv_writer::ResolvePath(ros_node_->get_logger(), csv_file_path);
  try {
    waypoint_csv_writer::WriteSingleCheckpoint(
      resolved_path, target_x, target_y, target_z, z_mode, target_roll, target_yaw);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "WriteAxisOverrideWaypointCSV: CSV書き込みに失敗しました: %s",
      e.what());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "WriteAxisOverrideWaypointCSV: target=(%.2f, %.2f, %.2f, yaw=%.1f, %s) を %s へ"
    "書き込みました",
    target_x, target_y, target_z, target_yaw, z_mode_str.c_str(), resolved_path.c_str());
  return BT::NodeStatus::SUCCESS;
}

void WriteAxisOverrideWaypointCSV::onHalted() {}

}  // namespace bluerov_control_bt_nodes
