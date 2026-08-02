#include "behavior_tree/publish_wrench_plan_reset.hpp"

namespace behavior_tree
{

PublishWrenchPlanReset::PublishWrenchPlanReset(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::SyncActionNode(name, config), ros_node_(ros_node)
{
  if (!getInput("topic_name", topic_name_)) {
    topic_name_ = "/planner/wrench_planner/goal_current_odom";
  }

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  publisher_ = ros_node_->create_publisher<planner_msgs::msg::WrenchPlan>(topic_name_, qos);
}

BT::PortsList PublishWrenchPlanReset::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "topic_name", "/planner/wrench_planner/goal_current_odom",
      "wrench_planner goal_current_odom topic"),
    BT::InputPort<int>(
      "reset_mask", 31, "PID reset bit mask (0b11111 resets x,y,z,roll,yaw)")};
}

BT::NodeStatus PublishWrenchPlanReset::tick()
{
  std::string topic_name;
  if (!getInput("topic_name", topic_name)) {
    topic_name = topic_name_;
  }

  if (topic_name != topic_name_) {
    topic_name_ = topic_name;
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    publisher_ = ros_node_->create_publisher<planner_msgs::msg::WrenchPlan>(topic_name_, qos);
  }

  int reset_mask = 31;
  if (!getInput("reset_mask", reset_mask)) {
    reset_mask = 31;
  }

  if (!publisher_) {
    RCLCPP_ERROR(ros_node_->get_logger(), "[%s] Publisher not initialized", this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto msg = planner_msgs::msg::WrenchPlan();
  msg.header.stamp = ros_node_->get_clock()->now();
  msg.reset = static_cast<uint8_t>(reset_mask & 0b00011111);
  msg.has_master = false;
  msg.has_slave = false;
  publisher_->publish(msg);

  RCLCPP_INFO(
    ros_node_->get_logger(), "[%s] Published WrenchPlan reset mask 0x%02x on %s",
    this->name().c_str(), msg.reset, topic_name_.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace behavior_tree
