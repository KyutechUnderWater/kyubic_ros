/**
 * @file depth.cpp
 * @brief Get depth data
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details 深度データを取得し，トピックに流す
 * reference: yusukemizoguchi on 22/06/26.
 *********************************************/

#include "sensors_esp32_driver/depth.hpp"

#include <protolink/udp_protocol.hpp>

#include "sensors_esp32_driver/common/utils.hpp"

using namespace std::chrono_literals;

namespace sensors_esp32_driver
{

Depth::Depth(const rclcpp::NodeOptions & options) : Node("depth", options)
{
  mcu_ip_addr = this->declare_parameter("mcu_ip_addr", "192.168.9.5");
  mcu_port = this->declare_parameter("mcu_port", 9000);
  this_port = this->declare_parameter("this_port", 9000);
  timeout_ms = this->declare_parameter("timeout_ms", 1000);

  try {
    sock_ = protolink::udp_protocol::create_socket(io_context_, this_port);
    RCLCPP_INFO(this->get_logger(), "Depth open port: %d", this_port);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize udp publisher: %s", e.what());
    RCLCPP_ERROR(this->get_logger(), "Depth don't open port %d", this_port);
    exit(1);
  }
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ms * 1e6);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::Depth>("depth", qos);
  sub_ = create_subscription<driver_msgs::msg::BoolStamped>(
    "depth_type", qos, std::bind(&Depth::ros_callback, this, std::placeholders::_1));

  protolink_publisher_ = std::make_shared<protolink::udp_protocol::Publisher<protoBoolStamped>>(
    sock_, mcu_ip_addr, mcu_port);
  protolink_subscriber_ = std::make_shared<protolink::udp_protocol::Subscriber<protoDepth>>(
    sock_, std::bind(&Depth::protolink_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(100ms, [this]() {
    if (timeout_ms) check_timeout<driver_msgs::msg::Depth>(this, mutex_, timeout_, pub_, "Depth");
  });
}

void Depth::ros_callback(const driver_msgs::msg::BoolStamped & _msg)
{
  protoBoolStamped proto_msg = protolink__driver_msgs__BoolStamped::convert(_msg);

  if (protolink_publisher_) {
    protolink_publisher_->send(proto_msg);

    RCLCPP_DEBUG(this->get_logger(), "Sent BoolStamped command via serial");
  } else {
    RCLCPP_WARN(this->get_logger(), "Protolink Publisher is not initialized");
  }
}

void Depth::protolink_callback(const protoDepth & _msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  timeout_->reset(this->get_clock()->now());

  auto msg =
    std::make_unique<driver_msgs::msg::Depth>(protolink__driver_msgs__Depth::convert(_msg));
  msg->header.stamp = this->get_clock()->now();
  msg->header.frame_id = "power_link";

  pub_->publish(std::move(msg));
  RCLCPP_INFO(
    this->get_logger(), "Publish ---> Depth: %2.2f  Temp: %2.2f", _msg.depth(), _msg.temperature());
}

}  // namespace sensors_esp32_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sensors_esp32_driver::Depth)
