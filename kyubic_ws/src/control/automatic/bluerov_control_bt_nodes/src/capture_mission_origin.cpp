#include "bluerov_control_bt_nodes/capture_mission_origin.hpp"

#include <common_msgs/msg/status.hpp>

namespace bluerov_control_bt_nodes
{

CaptureMissionOrigin::CaptureMissionOrigin(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::StatefulActionNode(name, config), ros_node_(ros_node)
{
  std::string odom_topic_name;
  if (!getInput("odom_topic_name", odom_topic_name)) {
    odom_topic_name = "odom";
  }

  odom_sub_ = ros_node_->create_subscription<localization_msgs::msg::Odometry>(
    odom_topic_name, 10,
    std::bind(&CaptureMissionOrigin::odomCallback, this, std::placeholders::_1));
}

BT::PortsList CaptureMissionOrigin::providedPorts()
{
  return {
    BT::InputPort<std::string>("odom_topic_name", "odom", "原点として確定するodomトピック")};
}

void CaptureMissionOrigin::odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  // status.dvl.id == NORMAL のodomのみ保持する。それ以外(DVLが未ロック、または
  // Part Aの修正でERROR時に直前値=起動直後は原点0が保持された値)は無視する。
  if (msg->status.dvl.id == common_msgs::msg::Status::NORMAL) {
    latest_normal_odom_ = msg;
  }
}

BT::NodeStatus CaptureMissionOrigin::onStart()
{
  latest_normal_odom_.reset();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CaptureMissionOrigin::onRunning()
{
  if (!latest_normal_odom_) {
    // status.dvl.id == NORMAL のodomをまだ受信していない。安全側に倒し、届くまで待機する
    // (呼び出し側XMLでTimeoutに包まれている前提)。
    return BT::NodeStatus::RUNNING;
  }

  const double origin_x = latest_normal_odom_->pose.position.x;
  const double origin_y = latest_normal_odom_->pose.position.y;
  config().blackboard->set("mission_origin_x", origin_x);
  config().blackboard->set("mission_origin_y", origin_y);

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "CaptureMissionOrigin: mission_origin=(%.2f, %.2f) を確定しました", origin_x, origin_y);
  return BT::NodeStatus::SUCCESS;
}

void CaptureMissionOrigin::onHalted() { latest_normal_odom_.reset(); }

}  // namespace bluerov_control_bt_nodes
