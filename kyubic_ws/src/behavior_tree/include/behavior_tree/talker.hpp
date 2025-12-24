/**
 * @file talker.hpp
 * @brief Publish string message to ros2 topic
 * @author R.ohishi
 * @date 2025/12/22
 *
 * @details BehaviorTreeのポートから文字列を取得し、std_msgs::msg::StringとしてPublishする
 **************************************************************/

#ifndef _TALKER_HPP
#define _TALKER_HPP

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace behavior_tree
{

/**
 * @brief Action node to publish a string message to a ROS 2 topic.
 * @details This node reads a string from the input port "message" and publishes it
 * to the topic specified in the input port "topic_name".
 */
class Talker : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor for the Talker node.
   * @param name The name of the node in the behavior tree.
   * @param config The configuration of the node.
   * @param ros_node Shared pointer to the ROS 2 node used for publishing.
   */
  Talker(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  /**
   * @brief Defines the input ports for this node.
   * @return A list containing:
   * - InputPort "message": The string content to publish.
   * - InputPort "topic_name": The ROS 2 topic to publish to (default: "output").
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Publishes the message to the topic.
   * @return BT::NodeStatus::SUCCESS if published successfully.
   * BT::NodeStatus::FAILURE if the message port is empty or publisher is invalid.
   */
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string topic_name_;
};

}  // namespace behavior_tree

#endif  // _TALKER_HPP
