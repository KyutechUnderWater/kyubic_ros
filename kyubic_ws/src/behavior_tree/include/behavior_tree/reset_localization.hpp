#ifndef _RESET_LOCALIZATION_HPP
#define _RESET_LOCALIZATION_HPP

#include <behaviortree_cpp/action_node.h>

#include <localization_msgs/srv/reset.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace behavior_tree
{

class ResetLocalization : public BT::StatefulActionNode
{
public:
  ResetLocalization(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub,
    rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub_;
  rclcpp::Client<localization_msgs::srv::Reset>::SharedPtr client_;
  std::shared_future<localization_msgs::srv::Reset::Response::SharedPtr> future_response_;
  std::string last_service_name_;
};

}  // namespace behavior_tree

#endif
