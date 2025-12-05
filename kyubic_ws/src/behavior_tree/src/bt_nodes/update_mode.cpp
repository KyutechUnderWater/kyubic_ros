/**
 * @file update_mode.cpp
 * @brief Get mode from joy-controller
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details Joy-Controllerからボタンの情報を取得しmodeを更新する
 **************************************************************/

#include "behavior_tree/update_mode.hpp"

namespace behavior_tree
{

UpdateMode::UpdateMode(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub, rclcpp::Node::SharedPtr ros_node,
  uint64_t timeout_ms)
: BT::SyncActionNode(name, config),
  ros_node_(ros_node),
  logger_pub_(logger_pub),
  timeout_ms_(timeout_ms)
{
  timeout_ = std::make_shared<timer::Timeout>(ros_node_->get_clock()->now(), timeout_ms * 1e6);

  std::string manual_name;
  if (!getInput("manual_button", manual_name)) {
    manual_name = "cross";
  }
  std::string auto_name;
  if (!getInput("auto_button", auto_name)) {
    auto_name = "circle";
  }

  button_map_ = joy_common::get_button_map();
  manual_it = button_map_.find(manual_name);
  auto_it = button_map_.find(auto_name);

  if (manual_it == button_map_.end()) {
    throw std::runtime_error("Invalid manual_button name: " + manual_name);
  }
  if (auto_it == button_map_.end()) {
    throw std::runtime_error("Invalid auto_button name: " + auto_name);
  }

  // Get topic name from the Input Port
  std::string topic_name;
  if (!getInput("topic_name", topic_name)) {
    topic_name = "/joy_common/joy_common";
    RCLCPP_WARN(
      ros_node_->get_logger(), "Failed to get 'joy_topic' from port, using default: %s",
      topic_name.c_str());
  }

  joy_sub_ = ros_node_->create_subscription<joy_common_msgs::msg::Joy>(
    topic_name, 10, [this](joy_common_msgs::msg::Joy::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      timeout_->reset(ros_node_->get_clock()->now());
      joy_msg_ = msg;
    });
}

BT::PortsList UpdateMode::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic_name", "/joy_common/joy_common", "Joystick topic name"),
    BT::InputPort<std::string>("manual_button", "Button name for manual mode"),
    BT::InputPort<std::string>("auto_button", "Button name for auto mode"),
    BT::OutputPort<std::string>("mode", "Current operation mode")};
}

BT::NodeStatus UpdateMode::tick()
{
  std::lock_guard<std::mutex> lock(mutex_);
  // If timeout
  if (timeout_->is_timeout(ros_node_->get_clock()->now())) {
    return BT::NodeStatus::FAILURE;
  }

  if (!joy_msg_) {
    setOutput<std::string>("mode", "manual");
    return BT::NodeStatus::SUCCESS;
  }

  auto manual_val = manual_it->second(joy_msg_->buttons);
  auto auto_val = auto_it->second(joy_msg_->buttons);

  bool manual_active = _check_value(manual_val);
  bool auto_active = _check_value(auto_val);

  if (manual_active && !auto_active) {
    setOutput<std::string>("mode", "manual");
  } else if (!manual_active && auto_active) {
    setOutput<std::string>("mode", "auto");
  }

  return BT::NodeStatus::SUCCESS;
}

bool UpdateMode::_check_value(std::variant<bool, double> value)
{
  if (std::holds_alternative<bool>(value)) {
    return std::get<bool>(value) == true;
  } else if (std::holds_alternative<double>(value)) {
    return std::get<double>(value) >= 1.0;
  }
  return false;
}

}  // namespace behavior_tree
