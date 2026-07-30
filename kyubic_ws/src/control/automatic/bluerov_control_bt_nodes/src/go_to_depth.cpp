#include "bluerov_control_bt_nodes/go_to_depth.hpp"

#include <cmath>

namespace bluerov_control_bt_nodes
{

GoToDepth::GoToDepth(
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
    odom_topic_name, 10, std::bind(&GoToDepth::odomCallback, this, std::placeholders::_1));

  // XML側でdepth_absolute_m/depth_offset_mが両方省略された場合のフォールバック。
  if (!ros_node_->has_parameter("default_depth_m")) {
    ros_node_->declare_parameter<double>("default_depth_m", 7.0);
  }
}

BT::PortsList GoToDepth::providedPorts()
{
  return {
    BT::InputPort<double>("depth_absolute_m", "絶対深度目標[m](NED、正=下方向)。優先"),
    BT::InputPort<double>(
      "depth_offset_m", "起動時点の深度からの相対オフセット[m](符号込み、正=さらに潜る)"),
    BT::InputPort<double>("tolerance_m", 0.1, "到達とみなす深度誤差[m]"),
    BT::InputPort<std::string>("topic_name", "zoh_wrench_plan", "publish先のWrenchPlanトピック"),
    BT::InputPort<std::string>("odom_topic_name", "odom", "現在のx/y/roll/yaw/深度を読むodomトピック")};
}

void GoToDepth::odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

BT::NodeStatus GoToDepth::onStart()
{
  published_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToDepth::onRunning()
{
  if (!latest_odom_) {
    // odomをまだ一度も受信していない。安全側に倒し、届くまで待機する。
    return BT::NodeStatus::RUNNING;
  }

  if (!published_) {
    double depth_absolute_m = 0.0;
    double depth_offset_m = 0.0;
    const bool has_absolute = static_cast<bool>(getInput("depth_absolute_m", depth_absolute_m));
    const bool has_offset = static_cast<bool>(getInput("depth_offset_m", depth_offset_m));

    const double current_depth = latest_odom_->pose.position.z_depth;
    if (has_absolute) {
      target_depth_m_ = depth_absolute_m;
    } else if (has_offset) {
      target_depth_m_ = current_depth + depth_offset_m;
    } else {
      // depth_absolute_m/depth_offset_mどちらも未指定。ROSパラメータへフォールバックする
      // (Phase 1のSetDepthTarget/DESCEND_TO_7M相当との後方互換)。
      target_depth_m_ = ros_node_->get_parameter("default_depth_m").as_double();
    }

    planner_msgs::msg::WrenchPlan msg;
    msg.header.stamp = ros_node_->get_clock()->now();
    msg.z_mode = planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH;
    msg.has_master = false;  // masterはwrench_planner側がodomから自動補完(現在値フィードバック用)
    msg.has_slave = false;
    // targetsはwrench_planner側では自動補完されないため、x/y/roll/yawは現在地を維持するよう
    // 明示的にコピーする(深度だけを絶対値で変える)。
    msg.targets.x = latest_odom_->pose.position.x;
    msg.targets.y = latest_odom_->pose.position.y;
    msg.targets.roll = latest_odom_->pose.orientation.x;
    msg.targets.yaw = latest_odom_->pose.orientation.z;
    msg.targets.z = target_depth_m_;
    publisher_->publish(msg);

    RCLCPP_INFO(
      ros_node_->get_logger(), "GoToDepth: target_depth_m=%.2f を publish しました (x=%.2f, y=%.2f)",
      target_depth_m_, msg.targets.x, msg.targets.y);
    published_ = true;
    return BT::NodeStatus::RUNNING;
  }

  double tolerance_m = 0.1;
  getInput("tolerance_m", tolerance_m);
  const double current_depth = latest_odom_->pose.position.z_depth;
  if (std::abs(current_depth - target_depth_m_) <= tolerance_m) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void GoToDepth::onHalted() { published_ = false; }

}  // namespace bluerov_control_bt_nodes
