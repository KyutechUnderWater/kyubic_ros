#include "bluerov_control_bt_nodes/go_to_blackboard_target.hpp"

#include <cmath>

namespace bluerov_control_bt_nodes
{

GoToBlackboardTarget::GoToBlackboardTarget(
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
    odom_topic_name, 10,
    std::bind(&GoToBlackboardTarget::odomCallback, this, std::placeholders::_1));
}

BT::PortsList GoToBlackboardTarget::providedPorts()
{
  return {
    BT::InputPort<double>("target_x", "目標の絶対x座標[m](通常は{discovered_target_x}を渡す)"),
    BT::InputPort<double>("target_y", "目標の絶対y座標[m](通常は{discovered_target_y}を渡す)"),
    BT::InputPort<double>(
      "target_z", "目標の絶対深度z[m](NED、正=下方向。通常は{discovered_target_z}を渡す)"),
    BT::InputPort<double>("position_tolerance_m", 0.2, "水平方向の到達とみなす距離[m]"),
    BT::InputPort<double>("depth_tolerance_m", 0.1, "深度方向の到達とみなす距離[m]"),
    BT::InputPort<std::string>("topic_name", "zoh_wrench_plan", "publish先のWrenchPlanトピック"),
    BT::InputPort<std::string>("odom_topic_name", "odom", "現在のroll/yawを読むodomトピック")};
}

void GoToBlackboardTarget::odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

BT::NodeStatus GoToBlackboardTarget::onStart()
{
  published_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToBlackboardTarget::onRunning()
{
  if (!latest_odom_) {
    // odomをまだ一度も受信していない。安全側に倒し、届くまで待機する。
    return BT::NodeStatus::RUNNING;
  }

  if (!published_) {
    if (!getInput("target_x", target_x_) || !getInput("target_y", target_y_) ||
        !getInput("target_z", target_z_)) {
      // target_x/y/zが未指定(ブラックボードにまだ書き込まれていない等)。
      // 安全側に倒し、指定されるまで待機する(勝手にどこかへ向かわない)。
      return BT::NodeStatus::RUNNING;
    }

    planner_msgs::msg::WrenchPlan msg;
    msg.header.stamp = ros_node_->get_clock()->now();
    msg.z_mode = planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH;
    msg.has_master = false;
    msg.has_slave = false;
    msg.targets.x = target_x_;
    msg.targets.y = target_y_;
    msg.targets.z = target_z_;
    // roll/yawは起動時点の現在値を維持する(GoToDepthと同じ考え方)。
    msg.targets.roll = latest_odom_->pose.orientation.x;
    msg.targets.yaw = latest_odom_->pose.orientation.z;
    publisher_->publish(msg);

    RCLCPP_INFO(
      ros_node_->get_logger(), "GoToBlackboardTarget: target=(%.2f, %.2f, %.2f) を publish しました",
      target_x_, target_y_, target_z_);
    published_ = true;
    return BT::NodeStatus::RUNNING;
  }

  double position_tolerance_m = 0.2;
  double depth_tolerance_m = 0.1;
  getInput("position_tolerance_m", position_tolerance_m);
  getInput("depth_tolerance_m", depth_tolerance_m);

  const double dx = target_x_ - latest_odom_->pose.position.x;
  const double dy = target_y_ - latest_odom_->pose.position.y;
  const double dz = target_z_ - latest_odom_->pose.position.z_depth;
  const double horizontal_distance = std::hypot(dx, dy);

  if (horizontal_distance <= position_tolerance_m && std::abs(dz) <= depth_tolerance_m) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void GoToBlackboardTarget::onHalted() { published_ = false; }

}  // namespace bluerov_control_bt_nodes
