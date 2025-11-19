/**
 * @file waypoint_action.cpp
 * @brief Action client for PDLA Planner
 * @author R.Ohnishi
 * @date 2025/11/19
 *
 * @details PDLA Action ServerにCSVパスを送り、経路追従を実行するBTノード
 *************************************************************/

#include "behavior_tree/waypoint_action.hpp"

namespace behavior_tree
{

// コンストラクタ
WaypointAction::WaypointAction(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: RosActionNode(name, config, ros_node)
{
}

// ポート定義
BT::PortsList WaypointAction::providedPorts()
{
  // [修正] providedBasicPorts は使わず、直接リストを返す
  return {
    BT::InputPort<std::string>("csv_file_path", "Path to the waypoint CSV file"),
    BT::InputPort<std::string>("action_name", "pdla_plan", "Action server name"),
    BT::OutputPort<uint32_t>("current_index", "Current executing waypoint index")};
}

// ゴール設定
// [修正] 型を明示的に指定 (planner_msgs::action::PDLA::Goal)
bool WaypointAction::setGoal(planner_msgs::action::PDLA::Goal & goal)
{
  auto res = getInput<std::string>("csv_file_path");
  if (!res) {
    // [修正] node_ -> ros_node_
    RCLCPP_ERROR(ros_node_->get_logger(), "WaypointAction: Input Port 'csv_file_path' is missing");
    return false;
  }

  goal.csv_file_path = res.value();

  // [修正] node_ -> ros_node_
  RCLCPP_INFO(
    ros_node_->get_logger(), "WaypointAction: Sending Goal -> %s", goal.csv_file_path.c_str());
  return true;
}

// 結果受信時の処理
BT::NodeStatus WaypointAction::onResult(const WrappedResult & wr)
{
  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result->success) {
    // [修正] node_ -> ros_node_
    RCLCPP_INFO(ros_node_->get_logger(), "WaypointAction: Succeeded");
    return BT::NodeStatus::SUCCESS;
  } else {
    // [修正] node_ -> ros_node_
    RCLCPP_ERROR(ros_node_->get_logger(), "WaypointAction: Failed (Code: %d)", (int)wr.code);
    return BT::NodeStatus::FAILURE;
  }
}

// フィードバック受信時の処理
void WaypointAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  setOutput("current_index", feedback->current_waypoint_index);

  // [修正] node_ -> ros_node_
  RCLCPP_INFO_THROTTLE(
    ros_node_->get_logger(), *ros_node_->get_clock(), 2000,
    "WaypointAction Feedback: Current Index %u", feedback->current_waypoint_index);
}

}  // namespace behavior_tree