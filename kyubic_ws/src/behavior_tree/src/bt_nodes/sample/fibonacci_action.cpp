/**
 * @file fibonacci_action.cpp
 * @brief Action client of fibonacci calculation
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details 公式サンプルのFibonacci Action Serverのクライアント
 *************************************************************/

#include "behavior_tree/sample/fibonacci_action.hpp"

namespace behavior_tree
{

FibonacciAction::FibonacciAction(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: RosActionNode(name, config, ros_node)
{
}

BT::PortsList FibonacciAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("action_name", "Action server name"),
    BT::InputPort<int>("order", "Fibonacci sequence order"),
    BT::OutputPort<std::vector<int32_t>>("result_sequence", "Result sequence")};
}

bool FibonacciAction::setGoal(typename Action::Goal & goal)
{
  auto order = getInput<int>("order");
  if (!order) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Input Port 'order' is missing");
    return false;
  }

  goal.order = order.value();
  return true;
}

BT::NodeStatus FibonacciAction::onResult(const WrappedResult & wr)
{
  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(ros_node_->get_logger(), "Fibonacci Action Succeeded");

    // Set the result to the blackboard
    setOutput("result_sequence", wr.result->sequence);

    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "Fibonacci Action Failed with code: %d", (int)wr.code);
    return BT::NodeStatus::FAILURE;
  }
}

void FibonacciAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO(
    ros_node_->get_logger(), "Feedback: Received partial sequence of size %zu",
    feedback->partial_sequence.size());
}

}  // namespace behavior_tree
