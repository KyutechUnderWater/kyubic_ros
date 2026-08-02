#include "bluerov_control_bt_nodes/check_pinger_range.hpp"

#include <cmath>

namespace bluerov_control_bt_nodes
{

namespace
{
constexpr double kDefaultScoreThreshold = 1.0e6;
}  // namespace

CheckPingerRange::CheckPingerRange(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::ConditionNode(name, config), ros_node_(ros_node)
{
  std::string position_topic = "pinger_position";
  std::string odom_topic_name = "odom";
  getInput("pinger_position_topic", position_topic);
  getInput("odom_topic_name", odom_topic_name);

  position_sub_ = ros_node_->create_subscription<planner_msgs::msg::PingerPosition>(
    position_topic, 10,
    std::bind(&CheckPingerRange::positionCallback, this, std::placeholders::_1));
  odom_sub_ = ros_node_->create_subscription<localization_msgs::msg::Odometry>(
    odom_topic_name, 10, std::bind(&CheckPingerRange::odomCallback, this, std::placeholders::_1));
}

BT::PortsList CheckPingerRange::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "pinger_position_topic", "pinger_position", "別ラズパイが発行する位置トピック(相対名。launchでremap)"),
    BT::InputPort<std::string>("odom_topic_name", "odom", "現在位置を読むodomトピック"),
    BT::InputPort<double>("range_threshold_m", 1.0, "これ以下なら十分近いとみなす[m]"),
    BT::InputPort<double>(
      "score_threshold", kDefaultScoreThreshold, "pinger_position.scoreの上限(これ以下なら位置を使用)")};
}

void CheckPingerRange::positionCallback(const planner_msgs::msg::PingerPosition::SharedPtr msg)
{
  latest_position_ = msg;
}

void CheckPingerRange::odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

BT::NodeStatus CheckPingerRange::tick()
{
  if (!latest_position_ || !latest_odom_) {
    return BT::NodeStatus::FAILURE;
  }

  double range_threshold_m = 1.0;
  double score_threshold = kDefaultScoreThreshold;
  getInput("range_threshold_m", range_threshold_m);
  getInput("score_threshold", score_threshold);
  if (latest_position_->score > score_threshold) {
    return BT::NodeStatus::FAILURE;
  }

  const double dx = latest_position_->x - latest_odom_->pose.position.x;
  const double dy = latest_position_->y - latest_odom_->pose.position.y;
  const double dz = latest_position_->z_depth - latest_odom_->pose.position.z_depth;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  return distance <= range_threshold_m ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace bluerov_control_bt_nodes
