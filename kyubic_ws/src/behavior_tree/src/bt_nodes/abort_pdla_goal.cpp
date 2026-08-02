#include "behavior_tree/abort_pdla_goal.hpp"

namespace behavior_tree
{

AbortPdlaGoal::AbortPdlaGoal(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub, rclcpp::Node::SharedPtr ros_node)
: BT::StatefulActionNode(name, config), ros_node_(ros_node), logger_pub_(logger_pub)
{
}

BT::PortsList AbortPdlaGoal::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "service_name", "/planner/pdla_planner/abort_active_goal",
      "PDLA abort_active_goal service name"),
    BT::InputPort<double>("timeout_sec", 2.0, "Service wait timeout (seconds)")};
}

BT::NodeStatus AbortPdlaGoal::onStart()
{
  auto service_name_opt = getInput<std::string>("service_name");
  auto timeout_sec_opt = getInput<double>("timeout_sec");

  if (!service_name_opt) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "[%s] Port 'service_name' is missing", this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::string service_name = service_name_opt.value();
  std::chrono::duration<double> timeout(timeout_sec_opt.value_or(2.0));

  if (!client_ || last_service_name_ != service_name) {
    client_ = ros_node_->create_client<std_srvs::srv::Trigger>(service_name);
    last_service_name_ = service_name;
  }

  if (!client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "[%s] Service '%s' not available", this->name().c_str(),
      service_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    ros_node_->get_logger(), "[%s] Calling PDLA abort: %s", this->name().c_str(),
    service_name.c_str());

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  future_response_ = client_->async_send_request(request).future.share();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AbortPdlaGoal::onRunning()
{
  if (future_response_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    try {
      auto response = future_response_.get();
      if (response->success) {
        RCLCPP_INFO(
          ros_node_->get_logger(), "[%s] PDLA abort successful: %s", this->name().c_str(),
          response->message.c_str());
        return BT::NodeStatus::SUCCESS;
      }
      RCLCPP_ERROR(
        ros_node_->get_logger(), "[%s] PDLA abort failed: %s", this->name().c_str(),
        response->message.c_str());
      return BT::NodeStatus::FAILURE;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        ros_node_->get_logger(), "[%s] Service call exception: %s", this->name().c_str(), e.what());
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void AbortPdlaGoal::onHalted()
{
  RCLCPP_WARN(ros_node_->get_logger(), "[%s] Halted", this->name().c_str());
}

}  // namespace behavior_tree
