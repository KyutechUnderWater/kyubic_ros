/**
 * @file wrench_action.cpp
 * @brief wrench action sample
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details 指定時間だけWrenchStamped型のtopicを流すAction
 ********************************************************/

#include "behavior_tree/sample/wrench_action.hpp"

namespace behavior_tree
{

WrenchAction::WrenchAction(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: RosActionNode<wrench_action_sample_msgs::action::Wrench>(name, config, ros_node)
{
}

BT::PortsList WrenchAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("action_name", "action server name"),
    BT::InputPort<int>("duration_sec", "Duration to apply wrench"),
    BT::OutputPort<uint32_t>("total_updates", "Total updates published")};
}

bool WrenchAction::setGoal(Goal & goal)
{
  auto duration = getInput<int>("duration_sec");
  if (!duration) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Input Port 'duration_sec' is missing");
    return false;
  }
  goal.duration_sec = duration.value();
  return true;
}

BT::NodeStatus WrenchAction::onResult(const WrappedResult & wr)
{
  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(ros_node_->get_logger(), "Action succeeded");
    setOutput("total_updates", wr.result->total_updates);
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN(ros_node_->get_logger(), "Action failed (Code: %d)", (int)wr.code);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace behavior_tree
