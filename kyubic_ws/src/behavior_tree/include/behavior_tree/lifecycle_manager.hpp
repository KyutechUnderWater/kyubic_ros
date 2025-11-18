/**
 * @file lifecycle_manager.hpp
 * @brief lifecycle node manager
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details LifecycleNodeを管理するBTノード
 *****************************************/

#ifndef _LIFECYCLE_MANAGER_HPP
#define _LIFECYCLE_MANAGER_HPP

#include <behaviortree_cpp/action_node.h>

#include <lifecycle_msgs/srv/change_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace behavior_tree
{

/**
 * @brief ROS 2 Lifecycle Node Management Action for BehaviorTree.CPP
 * @details Requests a state transition specified by the "transition" port for the lifecycle node
 *          specified by the "node_name" port.
 */
class LifecycleManager : public BT::StatefulActionNode
{
public:
  /**
   * @brief Constructor
   * @param name Node name
   * @param config Node configuration
   * @param ros_node Shared pointer to the ROS 2 node
   */
  LifecycleManager(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  /**
   * @brief Defines the ports used by this node
   * @return List of provided ports
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Called first when the node is ticked (starts async operation)
   * @return BT::NodeStatus::RUNNING
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Called continuously after onStart() until async operation completes
   * @return BT::NodeStatus::SUCCESS, FAILURE, or RUNNING
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief Called when the node is halted
   */
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;

  std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr> future_response_;

  std::string last_service_name_;
  std::map<std::string, uint8_t> transition_map_;
};

}  // namespace behavior_tree

#endif
