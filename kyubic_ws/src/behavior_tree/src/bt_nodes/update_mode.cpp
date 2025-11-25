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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub, rclcpp::Node::SharedPtr ros_node)
: BT::SyncActionNode(name, config), ros_node_(ros_node), logger_pub_(logger_pub)
{
  button_map_ = joy_common::get_button_map();

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
  if (!joy_msg_) {
    return BT::NodeStatus::FAILURE;
  }

  BT::Expected<std::string> manual_name = getInput<std::string>("manual_button");
  BT::Expected<std::string> auto_name = getInput<std::string>("auto_button");
  if (!manual_name || !auto_name) {
    return BT::NodeStatus::FAILURE;
  }

  auto manual_it = button_map_.find(manual_name.value());
  auto auto_it = button_map_.find(auto_name.value());
  if (manual_it == button_map_.end() || auto_it == button_map_.end()) {
    return BT::NodeStatus::FAILURE;
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
