/**
 * @file rgb.cpp
 * @brief neopixel rgb driver
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details NeoPixelに入力するRGBの値をSensor基板に送信
 *****************************************************/

#include "sensors_esp32_driver/rgb.hpp"

#include <protolink/udp_protocol.hpp>

namespace driver::sensors_esp32_driver
{

RGB::RGB(const rclcpp::NodeOptions & options) : rclcpp::Node("rgb", options)
{
  mcu_ip_addr = this->declare_parameter("mcu_ip_addr", "192.168.9.5");
  mcu_port = this->declare_parameter("mcu_port", 9000);
  this_port = this->declare_parameter("this_port", 9000);

  try {
    sock_ = protolink::udp_protocol::create_socket(io_context_, this_port);
    RCLCPP_INFO(this->get_logger(), "RGB open port: %d", this_port);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize udp publisher: %s", e.what());
    RCLCPP_ERROR(this->get_logger(), "RGB don't open port %d", this_port);
    exit(1);
  }

  protolink_publisher_ = std::make_shared<protolink::udp_protocol::Publisher<ProtoInt32Stamped>>(
    sock_, mcu_ip_addr, mcu_port, this->get_logger());

  sub_ = this->create_subscription<driver_msgs::msg::Int32Stamped>(
    "rgb", 10, [this](const driver_msgs::msg::Int32Stamped & _msg) {
      protolink_publisher_->send(protolink__driver_msgs__Int32Stamped::convert(_msg));
      RCLCPP_DEBUG(this->get_logger(), "Protolink Publish  --->  data: %d\n", _msg.data);
    });
}

}  // namespace driver::sensors_esp32_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(driver::sensors_esp32_driver::RGB)
