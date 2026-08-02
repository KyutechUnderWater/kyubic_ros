#include "bluerov_control_bt_nodes/write_pinger_waypoint_csv.hpp"

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
constexpr double kDefaultScoreThreshold = 1.0e6;

bool IsPositionUsable(
  const planner_msgs::msg::PingerPosition::SharedPtr & position, double score_threshold)
{
  return position && position->score <= score_threshold;
}

bool ComputeDirectionStepTarget(
  const planner_msgs::msg::PingerDirection::SharedPtr & direction,
  const localization_msgs::msg::Odometry::SharedPtr & odom, double step_distance_m, double & target_x,
  double & target_y, double & target_z, double & target_roll, double & target_yaw)
{
  const double pinger_yaw_rad = direction->yaw * kDegreeToRadian;
  const double pinger_pitch_rad = direction->pitch * kDegreeToRadian;
  const double body_offset_x =
    step_distance_m * std::cos(pinger_pitch_rad) * std::cos(pinger_yaw_rad);
  const double body_offset_y =
    step_distance_m * std::cos(pinger_pitch_rad) * std::sin(pinger_yaw_rad);
  const double body_offset_z = -step_distance_m * std::sin(pinger_pitch_rad);

  tf2::Quaternion q_rot;
  q_rot.setRPY(
    odom->pose.orientation.x * kDegreeToRadian, odom->pose.orientation.y * kDegreeToRadian,
    odom->pose.orientation.z * kDegreeToRadian);
  const tf2::Vector3 ned_offset = tf2::quatRotate(q_rot, tf2::Vector3(body_offset_x, body_offset_y, body_offset_z));

  target_x = odom->pose.position.x + ned_offset.x();
  target_y = odom->pose.position.y + ned_offset.y();
  target_z = odom->pose.position.z_depth + ned_offset.z();
  target_roll = odom->pose.orientation.x;
  target_yaw = (step_distance_m == 0.0) ? odom->pose.orientation.z + direction->yaw
                                        : odom->pose.orientation.z;
  return true;
}

bool ComputePositionStepTarget(
  const planner_msgs::msg::PingerPosition::SharedPtr & position,
  const localization_msgs::msg::Odometry::SharedPtr & odom, double step_distance_m, double & target_x,
  double & target_y, double & target_z, double & target_roll, double & target_yaw)
{
  const double dx = position->x - odom->pose.position.x;
  const double dy = position->y - odom->pose.position.y;
  const double dz = position->z_depth - odom->pose.position.z_depth;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (distance < 1.0e-6) {
    target_x = odom->pose.position.x;
    target_y = odom->pose.position.y;
    target_z = odom->pose.position.z_depth;
    target_roll = odom->pose.orientation.x;
    target_yaw = odom->pose.orientation.z;
    return true;
  }

  const double step = std::min(step_distance_m, distance);
  const double scale = step / distance;
  target_x = odom->pose.position.x + dx * scale;
  target_y = odom->pose.position.y + dy * scale;
  target_z = odom->pose.position.z_depth + dz * scale;
  target_roll = odom->pose.orientation.x;
  target_yaw = odom->pose.orientation.z;
  return true;
}
}  // namespace

WritePingerWaypointCSV::WritePingerWaypointCSV(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::StatefulActionNode(name, config), ros_node_(ros_node)
{
  std::string position_topic = "pinger_position";
  std::string direction_topic = "pinger_direction";
  std::string odom_topic_name = "odom";
  getInput("pinger_position_topic", position_topic);
  getInput("pinger_direction_topic", direction_topic);
  getInput("odom_topic_name", odom_topic_name);

  position_sub_ = ros_node_->create_subscription<planner_msgs::msg::PingerPosition>(
    position_topic, 10,
    std::bind(&WritePingerWaypointCSV::positionCallback, this, std::placeholders::_1));
  direction_sub_ = ros_node_->create_subscription<planner_msgs::msg::PingerDirection>(
    direction_topic, 10,
    std::bind(&WritePingerWaypointCSV::directionCallback, this, std::placeholders::_1));
  odom_sub_ = ros_node_->create_subscription<localization_msgs::msg::Odometry>(
    odom_topic_name, 10,
    std::bind(&WritePingerWaypointCSV::odomCallback, this, std::placeholders::_1));
}

BT::PortsList WritePingerWaypointCSV::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "pinger_position_topic", "pinger_position", "別ラズパイが発行する位置トピック(相対名。launchでremap)"),
    BT::InputPort<std::string>(
      "pinger_direction_topic", "pinger_direction", "別ラズパイが発行する方向トピック(相対名。launchでremap)"),
    BT::InputPort<std::string>("odom_topic_name", "odom", "現在姿勢を読むodomトピック"),
    BT::InputPort<double>("step_distance_m", 0.5, "1回で進む距離[m]"),
    BT::InputPort<double>(
      "score_threshold", kDefaultScoreThreshold, "pinger_position.scoreの上限(これ以下なら位置を使用)"),
    BT::InputPort<std::string>(
      "csv_file_path", "書き込むCSVパス('/'始まりでなければpath_plannerのshareディレクトリ基準)")};
}

void WritePingerWaypointCSV::positionCallback(
  const planner_msgs::msg::PingerPosition::SharedPtr msg)
{
  latest_position_ = msg;
}

void WritePingerWaypointCSV::directionCallback(
  const planner_msgs::msg::PingerDirection::SharedPtr msg)
{
  latest_direction_ = msg;
}

void WritePingerWaypointCSV::odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

BT::NodeStatus WritePingerWaypointCSV::onStart() { return BT::NodeStatus::RUNNING; }

BT::NodeStatus WritePingerWaypointCSV::onRunning()
{
  if (!latest_odom_) {
    return BT::NodeStatus::RUNNING;
  }

  double step_distance_m = 0.5;
  double score_threshold = kDefaultScoreThreshold;
  getInput("step_distance_m", step_distance_m);
  getInput("score_threshold", score_threshold);

  std::string csv_file_path;
  if (!getInput("csv_file_path", csv_file_path) || csv_file_path.empty()) {
    RCLCPP_ERROR(ros_node_->get_logger(), "WritePingerWaypointCSV: 'csv_file_path' は必須です");
    return BT::NodeStatus::FAILURE;
  }

  double target_x = 0.0;
  double target_y = 0.0;
  double target_z = 0.0;
  double target_roll = 0.0;
  double target_yaw = 0.0;
  const bool use_position = IsPositionUsable(latest_position_, score_threshold);
  if (use_position) {
    ComputePositionStepTarget(
      latest_position_, latest_odom_, step_distance_m, target_x, target_y, target_z, target_roll,
      target_yaw);
  } else if (latest_direction_) {
    ComputeDirectionStepTarget(
      latest_direction_, latest_odom_, step_distance_m, target_x, target_y, target_z, target_roll,
      target_yaw);
  } else {
    return BT::NodeStatus::RUNNING;
  }

  const std::string resolved_path =
    waypoint_csv_writer::ResolvePath(ros_node_->get_logger(), csv_file_path);
  try {
    waypoint_csv_writer::WriteSingleCheckpoint(
      resolved_path, target_x, target_y, target_z, planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH,
      target_roll, target_yaw);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "WritePingerWaypointCSV: CSV書き込みに失敗しました: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "WritePingerWaypointCSV: mode=%s target=(%.2f, %.2f, %.2f) -> %s",
    use_position ? "position" : "direction", target_x, target_y, target_z, resolved_path.c_str());
  return BT::NodeStatus::SUCCESS;
}

void WritePingerWaypointCSV::onHalted() {}

}  // namespace bluerov_control_bt_nodes
