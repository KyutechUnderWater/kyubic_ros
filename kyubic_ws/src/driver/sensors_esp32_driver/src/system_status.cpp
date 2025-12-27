/**
 * @file system_status.cpp
 * @brief button battery level check
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details ボタン電池残量に関するデータを取得し，トピックに流す
 **************************************************************/

#include "sensors_esp32_driver/system_status.hpp"

#include <protolink/udp_protocol.hpp>

#include "sensors_esp32_driver/common/utils.hpp"

using namespace std::chrono_literals;

namespace sensors_esp32_driver
{

SystemStatus::SystemStatus(const rclcpp::NodeOptions & options) : Node("system_status", options)
{
  sub_port = this->declare_parameter("sub_port", 9000);
  timeout_ms = this->declare_parameter("timeout_ms", 1000);

  try {
    sock_ = protolink::udp_protocol::create_socket(io_context_, sub_port);
    RCLCPP_INFO(this->get_logger(), "SystemStatus open port: %d", sub_port);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize udp publisher: %s", e.what());
    RCLCPP_ERROR(this->get_logger(), "SystemStatus don't open port %d", sub_port);
    exit(1);
  }
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ms * 1e6);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::SystemStatus>("system_status", rclcpp::SensorDataQoS());

  protolink_subscriber_ = std::make_shared<protolink::udp_protocol::Subscriber<ProtoSystemStatus>>(
    sock_, std::bind(&SystemStatus::protolink_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(std::chrono::milliseconds(timeout_ms / 2), [this]() {
    if (timeout_ms)
      check_timeout<driver_msgs::msg::SystemStatus>(this, mutex_, timeout_, pub_, "SystemStatus");
  });
}

void SystemStatus::protolink_callback(const ProtoSystemStatus & _msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    timeout_->reset(this->get_clock()->now());
  }

  auto msg = std::make_unique<driver_msgs::msg::SystemStatus>(
    protolink__driver_msgs__SystemStatus::convert(_msg));
  msg->header.stamp = this->get_clock()->now();
  msg->header.frame_id = "system_status";

  pub_->publish(std::move(msg));
  RCLCPP_DEBUG(
    this->get_logger(), "Publish ---> jetson: %d, actuator: %d, logic_relay: %d, usb: %d",
    _msg.jetson(), _msg.actuator_power(), _msg.logic_relay(), _msg.usb_power());
}

}  // namespace sensors_esp32_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sensors_esp32_driver::SystemStatus)
