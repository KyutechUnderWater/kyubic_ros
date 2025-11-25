/**
 * @file check_battery_state.cpp
 * @brief Check battery voltage state
 * @author J.Miura
 * @date 2025/11/25
 *
 * @details バッテリ電圧を取得し、4セル/6セルを自動判別して低電圧チェックを行う
 *****************************************************************************/

#include "behavior_tree/check/battery_check.hpp"

BatteryCheck::BatteryCheck(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub, rclcpp::Node::SharedPtr ros_node)
: BT::ConditionNode(name, config),
  ros_node_(ros_node),
  logger_pub_(logger_pub),
  logic_voltage_(0.0),
  act_voltage_(0.0)
{
  power_sub_ = ros_node_->create_subscription<driver_msgs::msg::PowerState>(
    "power_state", 10, [this](driver_msgs::msg::PowerState::SharedPtr msg) {
      logic_voltage_ = msg->log_voltage;
      act_voltage_ = msg->act_voltage;
    });
}
BT::PortsList BatteryCheck::providedPorts() { return {}; }

BT::NodeStatus BatteryCheck::tick()
{
  bool logic_check = is_battery_good(logic_voltage_);
  bool act_check = is_battery_good(act_voltage_);

  logger(logic_check, act_check);

  if (logic_check && act_check) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

bool BatteryCheck::is_battery_good(float voltage)
{
  if (voltage > 16.8f) {  // [6S] limit: 19.8V (3.3V * 6)
    if (voltage < 19.8f) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Low Battery (6S): %.2f V < 19.8 V", voltage);
      return false;
    }
  } else {  // [4S or 0V] limit: 13.2V (3.3V * 4)
    if (voltage < 13.2f) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Low Battery (4S): %.2f V <= 13.2 V", voltage);
      return false;
    }
  }
  return true;
}

void BatteryCheck::logger(bool logic_check, bool act_check)
{
  std::string s = std::format(
    "logic({:.3f}V): {}, actuator({:3f}V): {}", logic_voltage_.load(), logic_check,
    act_voltage_.load(), act_check);

  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "[BatteryCheck] " + s;
  logger_pub_->publish(std::move(msg));
}
