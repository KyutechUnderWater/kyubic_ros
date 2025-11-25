/**
 * @file waypoint_action.cpp
 * @brief Action client for PDLA Planner
 * @author S.Itozono
 * @date 2025/11/19
 *
 * @details PDLA Action ServerにCSVパスを送り、経路追従を実行するBTノード
 ***********************************************************************/

#include "behavior_tree/waypoint_action.hpp"

#include <localization_msgs/msg/odometry.hpp>

namespace behavior_tree
{

WaypointAction::WaypointAction(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub, rclcpp::Node::SharedPtr ros_node)
: RosActionNode(name, config, ros_node), logger_pub_(logger_pub)
{
}

BT::PortsList WaypointAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("csv_file_path", "Path to the waypoint CSV file"),
    BT::InputPort<std::string>("action_name", "pdla_plan", "Action server name"),
  };
}

bool WaypointAction::setGoal(planner_msgs::action::PDLA::Goal & goal)
{
  auto res = getInput<std::string>("csv_file_path");
  if (!res) {
    RCLCPP_ERROR(ros_node_->get_logger(), "WaypointAction: Input Port 'csv_file_path' is missing");
    return false;
  }

  goal.csv_file_path = res.value();

  RCLCPP_DEBUG(
    ros_node_->get_logger(), "WaypointAction: Sending Goal -> %s", goal.csv_file_path.c_str());
  return true;
}

BT::NodeStatus WaypointAction::onResult(const WrappedResult & wr)
{
  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result->success) {
    RCLCPP_DEBUG(ros_node_->get_logger(), "WaypointAction: Succeeded");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "WaypointAction: Failed (Code: %d)", (int)wr.code);
    return BT::NodeStatus::SUCCESS;
  }
}

void WaypointAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  std::string s = std::format(
    "WaypointAction Feedback: Current Index {:2d}, x: {:.2f}, y: {:.2f}, z_depth: {:.2f}",
    feedback->current_waypoint_index, feedback->current_odom.pose.position.x,
    feedback->current_odom.pose.position.y, feedback->current_odom.pose.position.z_depth);

  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "[QrAction] " + s;
  RCLCPP_DEBUG_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 2000, "%s", s.c_str());
}

}  // namespace behavior_tree
