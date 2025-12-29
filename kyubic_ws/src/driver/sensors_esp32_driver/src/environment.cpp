/**
 * @file environment.cpp
 * @brief environment monitoring
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details 環境データを取得し，トピックに流す
 *********************************************/

#include "sensors_esp32_driver/environment.hpp"

#include <functional>
#include <protolink/udp_protocol.hpp>

#include "sensors_esp32_driver/common/utils.hpp"

using namespace std::chrono_literals;

namespace sensors_esp32_driver
{

Environment::Environment(const rclcpp::NodeOptions & options) : Node("environment", options)
{
  sub_port = this->declare_parameter("sub_port", 9000);
  timeout_ms = this->declare_parameter("timeout_ms", 1000);

  try {
    sock_ = protolink::udp_protocol::create_socket(io_context_, sub_port);
    RCLCPP_INFO(this->get_logger(), "Environment open port: %d", sub_port);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize udp publisher: %s", e.what());
    RCLCPP_ERROR(this->get_logger(), "Environment don't open port %d", sub_port);
    exit(1);
  }
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ms * 1e6);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::Environment>("environment", rclcpp::SensorDataQoS());

  protolink_subscriber_ = std::make_shared<protolink::udp_protocol::Subscriber<ProtoEnvironment>>(
    sock_, std::bind(&Environment::protolink_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(std::chrono::milliseconds(timeout_ms / 2), [this]() {
    if (timeout_ms)
      check_timeout<driver_msgs::msg::Environment>(this, mutex_, timeout_, pub_, "Environment");
  });
}

void Environment::protolink_callback(const ProtoEnvironment & _msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    timeout_->reset(this->get_clock()->now());
  }

  auto msg = std::make_unique<driver_msgs::msg::Environment>(
    protolink__driver_msgs__Environment::convert(_msg));
  msg->header.stamp = this->get_clock()->now();

  pub_->publish(std::move(msg));
  RCLCPP_DEBUG(
    this->get_logger(), "Publish ---> Temp: %.2f, Hum: %.2f, Press: %.2f", _msg.temperature(),
    _msg.humidity(), _msg.pressure());
}

}  // namespace sensors_esp32_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sensors_esp32_driver::Environment)
