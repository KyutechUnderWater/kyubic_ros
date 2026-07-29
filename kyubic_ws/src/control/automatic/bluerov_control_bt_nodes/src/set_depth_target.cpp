#include "bluerov_control_bt_nodes/set_depth_target.hpp"

namespace bluerov_control_bt_nodes
{

SetDepthTarget::SetDepthTarget(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::StatefulActionNode(name, config), ros_node_(ros_node)
{
  std::string topic_name;
  if (!getInput("topic_name", topic_name)) {
    topic_name = "zoh_wrench_plan";
  }
  std::string odom_topic_name;
  if (!getInput("odom_topic_name", odom_topic_name)) {
    odom_topic_name = "odom";
  }

  publisher_ = ros_node_->create_publisher<planner_msgs::msg::WrenchPlan>(topic_name, 10);
  odom_sub_ = ros_node_->create_subscription<localization_msgs::msg::Odometry>(
    odom_topic_name, 10, std::bind(&SetDepthTarget::odomCallback, this, std::placeholders::_1));
}

BT::PortsList SetDepthTarget::providedPorts()
{
  return {
    BT::InputPort<double>("depth_m", "目標深度[m](NED、正=下方向)。絶対値"),
    BT::InputPort<std::string>("topic_name", "zoh_wrench_plan", "publish先のWrenchPlanトピック"),
    BT::InputPort<std::string>("odom_topic_name", "odom", "現在のx/y/roll/yawを読むodomトピック")};
}

void SetDepthTarget::odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

BT::NodeStatus SetDepthTarget::onStart()
{
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetDepthTarget::onRunning()
{
  if (!latest_odom_) {
    // odomをまだ一度も受信していない。安全側に倒し、届くまで待機する。
    return BT::NodeStatus::RUNNING;
  }

  double depth_m = 0.0;
  if (!getInput("depth_m", depth_m)) {
    RCLCPP_ERROR(ros_node_->get_logger(), "SetDepthTarget: Missing required input 'depth_m'");
    return BT::NodeStatus::FAILURE;
  }

  planner_msgs::msg::WrenchPlan msg;
  msg.header.stamp = ros_node_->get_clock()->now();
  msg.z_mode = planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH;
  msg.has_master = false;  // masterはwrench_planner側がodomから自動補完(現在値フィードバック用)
  msg.has_slave = false;
  // targetsはwrench_planner側では自動補完されないため、x/y/roll/yawは現在地を維持するよう
  // 明示的にコピーする(depthだけを絶対値で変える)。
  msg.targets.x = latest_odom_->pose.position.x;
  msg.targets.y = latest_odom_->pose.position.y;
  msg.targets.roll = latest_odom_->pose.orientation.x;
  msg.targets.yaw = latest_odom_->pose.orientation.z;
  msg.targets.z = depth_m;
  publisher_->publish(msg);

  RCLCPP_INFO(
    ros_node_->get_logger(), "SetDepthTarget: depth_m=%.2f を publish しました (x=%.2f, y=%.2f)",
    depth_m, msg.targets.x, msg.targets.y);
  return BT::NodeStatus::SUCCESS;
}

void SetDepthTarget::onHalted() {}

}  // namespace bluerov_control_bt_nodes
