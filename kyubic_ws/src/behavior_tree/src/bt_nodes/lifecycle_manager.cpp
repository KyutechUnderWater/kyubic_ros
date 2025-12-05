/**
 * @file lifecycle_manager.cpp
 * @brief lifecycle node manage
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details LifecycleNodeを管理するBTノード
 *****************************************/

#include "behavior_tree/lifecycle_manager.hpp"

namespace behavior_tree
{

LifecycleManager::LifecycleManager(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub, rclcpp::Node::SharedPtr ros_node)
: BT::StatefulActionNode(name, config), ros_node_(ros_node), logger_pub_(logger_pub)
{
  // Mapping transition strings to Transition IDs
  transition_map_["configure"] = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
  transition_map_["activate"] = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
  transition_map_["deactivate"] = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
  transition_map_["cleanup"] = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;

  // Shutdown is special (ID differs by state)
  transition_map_["shutdown"] = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN;
  transition_map_["shutdown_active"] = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN;
  transition_map_["shutdown_inactive"] =
    lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN;
}

BT::PortsList LifecycleManager::providedPorts()
{
  return {
    BT::InputPort<std::string>("node_name", "Name of the target Lifecycle node"),
    BT::InputPort<std::string>("transition", "Transition to execute (configure, activate, etc.)"),
    BT::InputPort<double>("timeout_sec", 1.0, "Service wait timeout (seconds)")};
}

BT::NodeStatus LifecycleManager::onStart()
{
  // Get values from input ports
  auto node_name_opt = getInput<std::string>("node_name");
  auto transition_str_opt = getInput<std::string>("transition");
  auto timeout_sec_opt = getInput<double>("timeout_sec");

  if (!node_name_opt || !transition_str_opt) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "[%s] Port 'node_name' or 'transition' is not set",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::string node_name = *node_name_opt;
  std::string transition_str = *transition_str_opt;
  std::chrono::duration<double> timeout(timeout_sec_opt.value_or(1.0));

  // Dynamically build service name
  std::string service_name = "/" + node_name + "/change_state";

  // Recreate client only if target node changed or client is uninitialized
  if (!change_state_client_ || last_service_name_ != service_name) {
    RCLCPP_DEBUG(
      ros_node_->get_logger(), "[%s] Creating service client: %s", this->name().c_str(),
      service_name.c_str());
    change_state_client_ = ros_node_->create_client<lifecycle_msgs::srv::ChangeState>(service_name);
    last_service_name_ = service_name;
  }

  // Wait briefly for service availability
  if (!change_state_client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "[%s] Service '%s' not available (timeout)", this->name().c_str(),
      service_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Convert string to Transition ID
  if (transition_map_.find(transition_str) == transition_map_.end()) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "[%s] Unknown transition: %s", this->name().c_str(),
      transition_str.c_str());
    return BT::NodeStatus::FAILURE;
  }
  uint8_t transition_id = transition_map_[transition_str];

  // Create request
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition_id;

  RCLCPP_DEBUG(
    ros_node_->get_logger(), "[%s] Requesting transition '%s' (%d) for node '%s'",
    this->name().c_str(), transition_str.c_str(), transition_id, node_name.c_str());

  // Call service asynchronously
  future_response_ = change_state_client_->async_send_request(request).future.share();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LifecycleManager::onRunning()
{
  // Check if response has arrived (non-blocking)
  if (future_response_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    try {
      auto response = future_response_.get();
      if (response->success) {
        RCLCPP_DEBUG(ros_node_->get_logger(), "[%s] Transition successful", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
      } else {
        RCLCPP_WARN(
          ros_node_->get_logger(), "[%s] Transition failed (server returned false)",
          this->name().c_str());
        return BT::NodeStatus::FAILURE;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        ros_node_->get_logger(), "[%s] Exception during service call: %s", this->name().c_str(),
        e.what());
      return BT::NodeStatus::FAILURE;
    }
  } else {
    return BT::NodeStatus::RUNNING;
  }
}

void LifecycleManager::onHalted()
{
  // rclcpp future does not support explicit cancellation.
  // Waiting for future to scope out (destructor call) releases related resources.
  RCLCPP_WARN(ros_node_->get_logger(), "[%s] Halted", this->name().c_str());
}

}  // namespace behavior_tree
