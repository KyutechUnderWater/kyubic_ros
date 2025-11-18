/**
 * @file wrench_action.hpp
 * @brief wrench action sample
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details 指定時間だけWrenchStamped型のtopicを流すAction
 ********************************************************/

#ifndef _WRENCH_ACTION_HPP
#define _WRENCH_ACTION_HPP

#include <behavior_tree/common/ros_action_node.hpp>
#include <wrench_action_sample_msgs/action/wrench.hpp>

namespace behavior_tree
{

/**
 * @brief Behavior Tree node for the Wrench Action.
 * @details Send the WrenchStamped Topic for the specified period.
 *          If the action completes successfully, it returns SUCCESS.
 */
class WrenchAction : public RosActionNode<wrench_action_sample_msgs::action::Wrench>
{
public:
  using Goal = wrench_action_sample_msgs::action::Wrench::Goal;

  /**
   * @brief Constructor for the WrenchAction node.
   * @param name The name of the node in the behavior tree.
   * @param config The node configuration.
   * @param ros_node Shared pointer to the ROS 2 node used for communication.
   */
  WrenchAction(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  /**
   * @brief Defines the input and output ports for this node.
   * @return A list containing:
   *         - InputPort "action_name": Name of the action server.
   *         - InputPort "duration_sec": Duration to apply the wrench (in seconds).
   *         - OutputPort "total_updates": The total number of updates received from the server.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Configures the Goal message before sending it to the server.
   * @param goal The goal object to be populated.
   * @return true if the goal was set successfully, false if the input port is missing.
   */
  bool setGoal(Goal & goal) override;

  /**
   * @brief callback executed when the action result is received.
   * @param wr The wrapped result containing the status code and the result data.
   * @return BT::NodeStatus::SUCCESS if the action succeeded, BT::NodeStatus::FAILURE otherwise.
   */
  BT::NodeStatus onResult(const WrappedResult & wr) override;
};

}  // namespace behavior_tree

#endif
