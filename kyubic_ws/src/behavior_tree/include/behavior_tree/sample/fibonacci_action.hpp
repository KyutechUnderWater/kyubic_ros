/**
 * @file fibonacci_action.hpp
 * @brief Action client of fibonacci calculation
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details 公式サンプルのFibonacci Action Serverのクライアント
 *************************************************************/

#ifndef _FIBONACCI_ACTION_HPP
#define _FIBONACCI_ACTION_HPP

#include <action_tutorials_interfaces/action/fibonacci.hpp>
#include <behavior_tree/common/ros_action_node.hpp>

namespace behavior_tree
{

/**
 * @brief Fibonacci Action Client implementation using the RosActionNode template.
 */
class FibonacciAction : public RosActionNode<action_tutorials_interfaces::action::Fibonacci>
{
public:
  /**
   * @brief Constructor
   * @param name Node name
   * @param config Node configuration
   * @param ros_node Shared pointer to the ROS 2 node
   */
  FibonacciAction(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  /**
   * @brief Define ports required by this specific action.
   * @return List of ports
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of setGoal.
   * Gets the "order" from the input port and populates the goal message.
   */
  bool setGoal(typename Action::Goal & goal) override;

  /**
   * @brief Implementation of onResult.
   * Checks the result code and sets the output port if successful.
   */
  BT::NodeStatus onResult(const WrappedResult & wr) override;

  /**
   * @brief (Optional) Implementation of onFeedback.
   * Logs the partial sequence length.
   */
  void onFeedback(const std::shared_ptr<const Feedback> feedback) override;
};

}  // namespace behavior_tree

#endif  // _FIBONACCI_ACTION_HPP
