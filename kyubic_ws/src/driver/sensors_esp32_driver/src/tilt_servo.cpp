/**
 * @file tilt_servo.cpp
 * @brief servo driver for camera tilt
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details カメラのチルト用サーボモータの信号をSensor基板に送信
 **************************************************************/

#include "sensors_esp32_driver/tilt_servo.hpp"

#include <protolink/udp_protocol.hpp>

namespace driver::sensors_esp32_driver
{

TiltServo::TiltServo(const rclcpp::NodeOptions & options) : rclcpp::Node("tilt_servo", options)
{
  mcu_ip_addr = this->declare_parameter("mcu_ip_addr", "192.168.9.5");
  mcu_port = this->declare_parameter("mcu_port", 9000);
  this_port = this->declare_parameter("this_port", 9000);

  try {
    sock_ = protolink::udp_protocol::create_socket(io_context_, this_port);
    RCLCPP_INFO(this->get_logger(), "Tilt servo open port: %d", this_port);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize udp publisher: %s", e.what());
    RCLCPP_ERROR(this->get_logger(), "Tilt servo don't open port %d", this_port);
    exit(1);
  }

  protolink_publisher_ = std::make_shared<protolink::udp_protocol::Publisher<protoInt32Stamped>>(
    sock_, mcu_ip_addr, mcu_port, this->get_logger());

  sub_ = this->create_subscription<driver_msgs::msg::Int32Stamped>(
    "tilt_servo", 10, [this](const driver_msgs::msg::Int32Stamped & _msg) {
      protolink_publisher_->send(protolink__driver_msgs__Int32Stamped::convert(_msg));
      RCLCPP_DEBUG(this->get_logger(), "Protolink Publish  --->  data: %d\n", _msg.data);
    });
}

}  // namespace driver::sensors_esp32_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(driver::sensors_esp32_driver::TiltServo)
