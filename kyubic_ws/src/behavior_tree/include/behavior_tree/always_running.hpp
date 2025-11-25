/**
 * @file always_running.hpp
 * @brief Behavior Tree Node that always return running
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details 常にBT::NodeStatus::RUNNINGを返すBehaviorTreeのノード
 ****************************************************************/

#ifndef _ALWAYS_RUNNING_HPP
#define _ALWAYS_RUNNING_HPP

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace behavior_tree
{

/**
 * @brief A simple node that constantly returns RUNNING
 * @details Returns RUNNING when onStart() is called.
 *          Returns RUNNING when onRunning() is called.
 */
class AlwaysRunning : public BT::StatefulActionNode
{
public:
  /**
   * @brief Constructor
   * @param name Node name
   * @param config Node configuration
   */
  AlwaysRunning(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub);

  /**
   * @brief Defines the ports used by this node.
   * @return An empty list as this node requires no ports.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Called first when the node is ticked.
   * @return BT::NodeStatus::RUNNING immediately.
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Called continuously after onStart() returns RUNNING.
   * @return BT::NodeStatus::RUNNING forever.
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief Called when the node is halted.
   */
  void onHalted() override;

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub_;
};

}  // namespace behavior_tree

#endif  // !_ALWAYS_RUNNING_HPP
