/**
 * @file reset_localization.cpp
 * @brief Service client for resetting localization
 *************************************************************/

#include "behavior_tree/reset_localization.hpp"

namespace behavior_tree
{

ResetLocalization::ResetLocalization(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::StatefulActionNode(name, config), ros_node_(ros_node)
{
}

BT::PortsList ResetLocalization::providedPorts()
{
  return {// [設定] デフォルトサービス名は PDLA Planner のものに合わせておきます
          BT::InputPort<std::string>(
            "service_name", "/pdla_planner/reset_trigger", "Name of the reset service"),
          BT::InputPort<double>("timeout_sec", 1.0, "Service wait timeout (seconds)")};
}

BT::NodeStatus ResetLocalization::onStart()
{
  auto service_name_opt = getInput<std::string>("service_name");
  auto timeout_sec_opt = getInput<double>("timeout_sec");

  if (!service_name_opt) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "[%s] Port 'service_name' is missing", this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::string service_name = service_name_opt.value();
  std::chrono::duration<double> timeout(timeout_sec_opt.value_or(1.0));

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

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  RCLCPP_INFO(
    ros_node_->get_logger(), "[%s] Calling reset service: %s", this->name().c_str(),
    service_name.c_str());

  future_response_ = client_->async_send_request(request).future.share();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ResetLocalization::onRunning()
{
  if (future_response_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    try {
      auto response = future_response_.get();
      if (response->success) {
        RCLCPP_INFO(
          ros_node_->get_logger(), "[%s] Reset successful: %s", this->name().c_str(),
          response->message.c_str());
        return BT::NodeStatus::SUCCESS;
      } else {
        RCLCPP_WARN(
          ros_node_->get_logger(), "[%s] Reset failed: %s", this->name().c_str(),
          response->message.c_str());
        return BT::NodeStatus::FAILURE;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        ros_node_->get_logger(), "[%s] Service call exception: %s", this->name().c_str(), e.what());
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void ResetLocalization::onHalted()
{
  RCLCPP_WARN(ros_node_->get_logger(), "[%s] Halted", this->name().c_str());
}

}  // namespace behavior_tree