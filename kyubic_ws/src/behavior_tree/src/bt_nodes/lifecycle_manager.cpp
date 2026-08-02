/**
 * @file lifecycle_manager.cpp
 * @brief lifecycle node manage
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details LifecycleNodeを管理するBTノード
 *****************************************/

#include "behavior_tree/lifecycle_manager.hpp"

#include <lifecycle_msgs/msg/state.hpp>

namespace behavior_tree
{
namespace
{

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

}  // namespace

LifecycleManager::LifecycleManager(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub, rclcpp::Node::SharedPtr ros_node)
: BT::StatefulActionNode(name, config), ros_node_(ros_node), logger_pub_(logger_pub)
{
  transition_map_["configure"] = Transition::TRANSITION_CONFIGURE;
  transition_map_["activate"] = Transition::TRANSITION_ACTIVATE;
  transition_map_["deactivate"] = Transition::TRANSITION_DEACTIVATE;
  transition_map_["cleanup"] = Transition::TRANSITION_CLEANUP;

  transition_map_["shutdown"] = Transition::TRANSITION_ACTIVE_SHUTDOWN;
  transition_map_["shutdown_active"] = Transition::TRANSITION_ACTIVE_SHUTDOWN;
  transition_map_["shutdown_inactive"] = Transition::TRANSITION_INACTIVE_SHUTDOWN;
}

BT::PortsList LifecycleManager::providedPorts()
{
  return {
    BT::InputPort<std::string>("node_name", "Name of the target Lifecycle node"),
    BT::InputPort<std::string>("transition", "Transition to execute (configure, activate, etc.)"),
    BT::InputPort<double>("timeout_sec", 1.0, "Service wait timeout (seconds)")};
}

std::optional<uint8_t> LifecycleManager::getPrimaryState(
  const std::string & node_name, std::chrono::duration<double> timeout) const
{
  const std::string service_name = "/" + node_name + "/get_state";
  auto get_state_client = ros_node_->create_client<lifecycle_msgs::srv::GetState>(service_name);
  if (!get_state_client->wait_for_service(timeout)) {
    RCLCPP_WARN(
      ros_node_->get_logger(), "[%s] get_state service '%s' not available (timeout)",
      this->name().c_str(), service_name.c_str());
    return std::nullopt;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto future = get_state_client->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_WARN(
      ros_node_->get_logger(), "[%s] get_state timed out for '%s'", this->name().c_str(),
      service_name.c_str());
    return std::nullopt;
  }

  return future.get()->current_state.id;
}

bool LifecycleManager::isTransitionSatisfied(
  uint8_t current_state, const std::string & transition) const
{
  if (transition == "configure") {
    return current_state == State::PRIMARY_STATE_INACTIVE;
  }
  if (transition == "activate") {
    return current_state == State::PRIMARY_STATE_ACTIVE;
  }
  if (transition == "deactivate") {
    return current_state == State::PRIMARY_STATE_INACTIVE ||
           current_state == State::PRIMARY_STATE_UNCONFIGURED;
  }
  if (transition == "cleanup") {
    return current_state == State::PRIMARY_STATE_UNCONFIGURED;
  }
  return false;
}

std::optional<uint8_t> LifecycleManager::resolveTransitionId(
  uint8_t current_state, const std::string & transition) const
{
  if (isTransitionSatisfied(current_state, transition)) {
    return std::nullopt;
  }

  if (transition == "configure" && current_state == State::PRIMARY_STATE_ACTIVE) {
    return Transition::TRANSITION_DEACTIVATE;
  }
  if (transition == "activate" && current_state == State::PRIMARY_STATE_UNCONFIGURED) {
    return Transition::TRANSITION_CONFIGURE;
  }
  if (transition == "cleanup" && current_state == State::PRIMARY_STATE_ACTIVE) {
    return Transition::TRANSITION_DEACTIVATE;
  }

  const auto it = transition_map_.find(transition);
  if (it == transition_map_.end()) {
    return std::nullopt;
  }
  return it->second;
}

bool LifecycleManager::beginChangeState(
  const std::string & node_name, uint8_t transition_id, std::chrono::duration<double> timeout)
{
  const std::string service_name = "/" + node_name + "/change_state";

  if (!change_state_client_ || last_service_name_ != service_name) {
    change_state_client_ =
      ros_node_->create_client<lifecycle_msgs::srv::ChangeState>(service_name);
    last_service_name_ = service_name;
  }

  if (!change_state_client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "[%s] Service '%s' not available (timeout)", this->name().c_str(),
      service_name.c_str());
    return false;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition_id;

  future_response_ = change_state_client_->async_send_request(request).future.share();
  return true;
}

BT::NodeStatus LifecycleManager::onStart()
{
  pending_transition_after_success_.reset();

  auto node_name_opt = getInput<std::string>("node_name");
  auto transition_str_opt = getInput<std::string>("transition");
  auto timeout_sec_opt = getInput<double>("timeout_sec");

  if (!node_name_opt || !transition_str_opt) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "[%s] Port 'node_name' or 'transition' is not set",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  target_node_name_ = *node_name_opt;
  requested_transition_ = *transition_str_opt;
  const std::chrono::duration<double> timeout(timeout_sec_opt.value_or(1.0));

  if (transition_map_.find(requested_transition_) == transition_map_.end()) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "[%s] Unknown transition: %s", this->name().c_str(),
      requested_transition_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto current_state = getPrimaryState(target_node_name_, timeout);
  if (current_state) {
    if (isTransitionSatisfied(*current_state, requested_transition_)) {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "[%s] Transition '%s' already satisfied for '%s' (state=%u); skipping",
        this->name().c_str(), requested_transition_.c_str(), target_node_name_.c_str(),
        *current_state);
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    RCLCPP_WARN(
      ros_node_->get_logger(),
      "[%s] Could not read lifecycle state for '%s'; attempting transition '%s' anyway",
      this->name().c_str(), target_node_name_.c_str(), requested_transition_.c_str());
    // inactive な ZOH へ deactivate すると lifecycle が拒否する。状態読取失敗時は
    // active 想定で deactivate を送らずスキップする(初回起動・Configure 直後で多い)。
    if (requested_transition_ == "deactivate") {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "[%s] Skipping deactivate for '%s' (state unreadable; assuming not active)",
        this->name().c_str(), target_node_name_.c_str());
      return BT::NodeStatus::SUCCESS;
    }
  }

  const uint8_t state_for_resolve = current_state.value_or(State::PRIMARY_STATE_UNKNOWN);
  const auto transition_id = resolveTransitionId(state_for_resolve, requested_transition_);
  if (!transition_id) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "[%s] Unable to resolve transition '%s' for state %u",
      this->name().c_str(), requested_transition_.c_str(), state_for_resolve);
    return BT::NodeStatus::FAILURE;
  }

  if (
    requested_transition_ == "activate" && state_for_resolve == State::PRIMARY_STATE_UNCONFIGURED) {
    pending_transition_after_success_ = "activate";
  } else if (
    requested_transition_ == "cleanup" && state_for_resolve == State::PRIMARY_STATE_ACTIVE) {
    pending_transition_after_success_ = "cleanup";
  }

  RCLCPP_DEBUG(
    ros_node_->get_logger(), "[%s] Requesting transition id %u for '%s' (requested '%s')",
    this->name().c_str(), *transition_id, target_node_name_.c_str(),
    requested_transition_.c_str());

  if (!beginChangeState(target_node_name_, *transition_id, timeout)) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LifecycleManager::onRunning()
{
  if (future_response_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }

  try {
    auto response = future_response_.get();
    if (!response->success) {
      RCLCPP_WARN(
        ros_node_->get_logger(), "[%s] Transition failed (server returned false)",
        this->name().c_str());
      return BT::NodeStatus::FAILURE;
    }

    if (pending_transition_after_success_) {
      const std::string next_transition = *pending_transition_after_success_;
      pending_transition_after_success_.reset();

      const auto timeout_sec_opt = getInput<double>("timeout_sec");
      const std::chrono::duration<double> timeout(timeout_sec_opt.value_or(1.0));

      const auto current_state = getPrimaryState(target_node_name_, timeout);
      const uint8_t state_for_resolve =
        current_state.value_or(State::PRIMARY_STATE_UNKNOWN);
      const auto transition_id = resolveTransitionId(state_for_resolve, next_transition);
      if (!transition_id) {
        RCLCPP_DEBUG(
          ros_node_->get_logger(), "[%s] Chained transition '%s' already satisfied",
          this->name().c_str(), next_transition.c_str());
        return BT::NodeStatus::SUCCESS;
      }

      RCLCPP_DEBUG(
        ros_node_->get_logger(), "[%s] Chaining transition '%s' (id %u) for '%s'",
        this->name().c_str(), next_transition.c_str(), *transition_id,
        target_node_name_.c_str());

      if (!beginChangeState(target_node_name_, *transition_id, timeout)) {
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }

    RCLCPP_DEBUG(ros_node_->get_logger(), "[%s] Transition successful", this->name().c_str());
    return BT::NodeStatus::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "[%s] Exception during service call: %s", this->name().c_str(),
      e.what());
    return BT::NodeStatus::FAILURE;
  }
}

void LifecycleManager::onHalted()
{
  pending_transition_after_success_.reset();
  RCLCPP_WARN(ros_node_->get_logger(), "[%s] Halted", this->name().c_str());
}

}  // namespace behavior_tree
