/**
 * @file reset_localization.hpp
 * @brief Service client for resetting localization (Trigger)
 *************************************************************/

#ifndef _RESET_LOCALIZATION_HPP
#define _RESET_LOCALIZATION_HPP

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace behavior_tree
{

class ResetLocalization : public BT::StatefulActionNode
{
public:
  ResetLocalization(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr> future_response_;
  std::string last_service_name_;
};

}  // namespace behavior_tree

#endif