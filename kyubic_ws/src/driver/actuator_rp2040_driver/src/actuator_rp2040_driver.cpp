/**
 * @file actuator_rp2040_driver.cpp
 * @brief Thruster driver
 * @author K.Fujimoto
 * @date 2025/11/21
 *
 * @details スラスターの出力値を受け取り，制御マイコンに命令する
 */

#include "actuator_rp2040_driver/actuator_rp2040_driver.hpp"

#include <protolink/serial_protocol.hpp>

using namespace std::chrono_literals;

namespace actuator_rp2040_driver
{

ActuatorRP2040::ActuatorRP2040(const rclcpp::NodeOptions & options)
: Node("actuator_rp2040_driver", options)
{
  portname = this->declare_parameter("portname", "/dev/ttyUSB0");
  baudrate = this->declare_parameter("baudrate", 115200);

  try {
    port_ = protolink::serial_protocol::create_port(io_context_, portname, baudrate);
    RCLCPP_INFO(
      this->get_logger(), "ActuatorRP2040 open port: %s (%d)", portname.c_str(), baudrate);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize udp publisher: %s", e.what());
    RCLCPP_ERROR(
      this->get_logger(), "ActuatorRP2040 don't open port %s (%d)", portname.c_str(), baudrate);
    exit(1);
  }

  protolink_publisher_ = std::make_shared<protolink::serial_protocol::Publisher<ProtoActuator>>(
    port_, this->get_logger());

  thr_sub_ = this->create_subscription<driver_msgs::msg::Thruster>(
    "thruster", 10, std::bind(&ActuatorRP2040::thruster_callback, this, std::placeholders::_1));

  led_sub_ = this->create_subscription<driver_msgs::msg::LED>(
    "led", 10, std::bind(&ActuatorRP2040::led_callback, this, std::placeholders::_1));
}

void ActuatorRP2040::thruster_callback(driver_msgs::msg::Thruster::SharedPtr _msg)
{
  msg_buffer_.thrusters = *_msg;

  ProtoActuator msg = protolink__driver_msgs__Actuator::convert(msg_buffer_);

  // Send data with serial
  if (protolink_publisher_) {
    protolink_publisher_->send(msg);
  }

  RCLCPP_DEBUG(
    this->get_logger(), "Thruster Update -> T1:%.2f T2:%.2f T3:%.2f ", _msg->thr1, _msg->thr2,
    _msg->thr3);
}

void ActuatorRP2040::led_callback(driver_msgs::msg::LED::SharedPtr _msg)
{
  msg_buffer_.leds = *_msg;
  ProtoActuator msg = protolink__driver_msgs__Actuator::convert(msg_buffer_);

  // Send data with serial
  if (protolink_publisher_) {
    protolink_publisher_->send(msg);
  }
}

}  // namespace actuator_rp2040_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(actuator_rp2040_driver::ActuatorRP2040)
