/**
 * @file button_battery_state.cpp
 * @brief button battery level check
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details ボタン電池残量に関するデータを取得し，トピックに流す
 **************************************************************/

#include "sensors_esp32_driver/button_battery_state.hpp"

#include <protolink/udp_protocol.hpp>

#include "sensors_esp32_driver/common/utils.hpp"

using namespace std::chrono_literals;

namespace sensors_esp32_driver
{

ButtonBatteryState::ButtonBatteryState(const rclcpp::NodeOptions & options)
: Node("button_battery_state", options)
{
  sub_port = this->declare_parameter("sub_port", 9000);
  timeout_ms = this->declare_parameter("timeout_ms", 1000);

  try {
    sock_ = protolink::udp_protocol::create_socket(io_context_, sub_port);
    RCLCPP_INFO(this->get_logger(), "ButtonBatteryState open port: %d", sub_port);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize udp publisher: %s", e.what());
    RCLCPP_ERROR(this->get_logger(), "ButtonBatteryState don't open port %d", sub_port);
    exit(1);
  }
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ms * 1e6);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::ButtonBatteryState>(
    "button_battery_state", rclcpp::SensorDataQoS());

  protolink_subscriber_ =
    std::make_shared<protolink::udp_protocol::Subscriber<ProtoButtonBatteryState>>(
      sock_, std::bind(&ButtonBatteryState::protolink_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(std::chrono::milliseconds(timeout_ms / 4), [this]() {
    if (timeout_ms)
      check_timeout<driver_msgs::msg::ButtonBatteryState>(
        this, mutex_, timeout_, pub_, "ButtonBatteryState");
  });
}

void ButtonBatteryState::protolink_callback(const ProtoButtonBatteryState & _msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    timeout_->reset(this->get_clock()->now());
  }

  auto msg = std::make_unique<driver_msgs::msg::ButtonBatteryState>(
    protolink__driver_msgs__ButtonBatteryState::convert(_msg));
  msg->header.stamp = this->get_clock()->now();
  msg->header.frame_id = "battery";

  pub_->publish(std::move(msg));
  RCLCPP_DEBUG(
    this->get_logger(), "Publish ---> Leak: %d, RTC: %d", _msg.battery_leak(), _msg.battery_rtc());
}

}  // namespace sensors_esp32_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sensors_esp32_driver::ButtonBatteryState)
