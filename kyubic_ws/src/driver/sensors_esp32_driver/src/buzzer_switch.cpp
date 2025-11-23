/**
 * @file buzzer_switch.cpp
 * @brief buzzer on/off control
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details ブザーの制御
 *****************************/

#include "sensors_esp32_driver/buzzer_switch.hpp"

namespace sensors_esp32_driver
{

BuzzerSwitch::BuzzerSwitch(const rclcpp::NodeOptions & options) : Node("buzzer_switch", options)
{
  mcu_ip_addr = this->declare_parameter("mcu_ip_addr", "192.168.9.5");
  mcu_port = this->declare_parameter("mcu_port", 9000);
  this_port = this->declare_parameter("this_port", 9000);

  try {
    sock_ = protolink::udp_protocol::create_socket(io_context_, this_port);
    RCLCPP_INFO(this->get_logger(), "BuzzerSwitch open port: %d", this_port);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize udp publisher: %s", e.what());
    RCLCPP_ERROR(this->get_logger(), "BuzzerSwitch don't open port %d", this_port);
    exit(1);
  }

  protolink_publisher_ = std::make_shared<protolink::udp_protocol::Publisher<ProtoBuzzerSwitch>>(
    sock_, mcu_ip_addr, mcu_port, this->get_logger());

  sub_ = this->create_subscription<driver_msgs::msg::BuzzerSwitch>(
    "buzzer_switch", 10, [this](const driver_msgs::msg::BuzzerSwitch & _msg) {
      protolink_publisher_->send(protolink__driver_msgs__BuzzerSwitch::convert(_msg));

      RCLCPP_DEBUG(
        this->get_logger(), "Protolink Publish ---> Buzzer: %d, Stop: %d", _msg.buzzer,
        _msg.buzzer_stop);
    });
}

}  // namespace sensors_esp32_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sensors_esp32_driver::BuzzerSwitch)
