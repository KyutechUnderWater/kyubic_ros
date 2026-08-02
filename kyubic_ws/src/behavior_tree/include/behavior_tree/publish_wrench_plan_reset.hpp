#ifndef _PUBLISH_WRENCH_PLAN_RESET_HPP
#define _PUBLISH_WRENCH_PLAN_RESET_HPP

#include <behaviortree_cpp/action_node.h>

#include <planner_msgs/msg/wrench_plan.hpp>
#include <rclcpp/rclcpp.hpp>

namespace behavior_tree
{

class PublishWrenchPlanReset : public BT::SyncActionNode
{
public:
  PublishWrenchPlanReset(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<planner_msgs::msg::WrenchPlan>::SharedPtr publisher_;
  std::string topic_name_;
};

}  // namespace behavior_tree

#endif
