/**
 * @file leak.cpp
 * @brief leak sensor driver
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details leak sensorの情報を取得して，Topicに流す
 **************************************************/

#include "sensors_esp32_driver/leak.hpp"

#include <protolink/udp_protocol.hpp>

#include "sensors_esp32_driver/common/utils.hpp"

using namespace std::chrono_literals;

namespace sensors_esp32_driver
{

Leak::Leak(const rclcpp::NodeOptions & options) : rclcpp::Node("leak", options)
{
  sub_port = this->declare_parameter("sub_port", 9000);
  timeout_ms = this->declare_parameter("timeout_ms", 1000);

  try {
    sock_ = protolink::udp_protocol::create_socket(io_context_, sub_port);
    RCLCPP_INFO(this->get_logger(), "Leak open port: %d", sub_port);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize udp publisher: %s", e.what());
    RCLCPP_ERROR(this->get_logger(), "Leak don't open port %d", sub_port);
    exit(1);
  }
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ms * 1e6);

  pub_ = this->create_publisher<driver_msgs::msg::BoolStamped>("leak", rclcpp::SensorDataQoS());

  protolink_subscriber_ = std::make_shared<protolink::udp_protocol::Subscriber<ProtoBoolStamped>>(
    sock_, std::bind(&Leak::protolink_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(std::chrono::milliseconds(timeout_ms / 2), [this]() {
    if (timeout_ms)
      check_timeout<driver_msgs::msg::BoolStamped>(this, mutex_, timeout_, pub_, "Leak");
  });
}

void Leak::protolink_callback(const ProtoBoolStamped & _msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    timeout_->reset(this->get_clock()->now());
  }

  auto msg = std::make_unique<driver_msgs::msg::BoolStamped>(
    protolink__driver_msgs__BoolStamped::convert(_msg));
  msg->header.stamp = this->get_clock()->now();

  pub_->publish(std::move(msg));
  RCLCPP_DEBUG(this->get_logger(), "Protolink Publish  --->  data: %d\n", _msg.data());
}

}  // namespace sensors_esp32_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sensors_esp32_driver::Leak)
