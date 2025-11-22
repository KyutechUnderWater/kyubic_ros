/**
 * @file actuator_rp2040_driver.hpp
 * @brief Driver for rp2040 on Actuator board
 * @author K.Fujimoto
 * @date 2025/11/21
 *
 * @details スラスターの出力値を受け取り，制御マイコンに命令する
 */

#ifndef _ACTUATOR_RP2040_DRIVER_HPP
#define _ACTUATOR_RP2040_DRIVER_HPP

#include <driver_msgs/msg/actuator.hpp>
#include <memory>
#include <proto_files/conversion_driver_msgs__Actuator.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>

namespace actuator_rp2040_driver
{

/**
 * @brief actuator class
 */
class ActuatorRP2040 : public rclcpp::Node
{
public:
  /**
   * @brief Protolink & ROS-topic settng
   * @details Protolink setting and Define a subscriber
   */
  explicit ActuatorRP2040(const rclcpp::NodeOptions & options);

private:
  std::string portname;
  uint32_t baudrate;

  protolink::IoContext io_context_;
  std::shared_ptr<protolink::serial_protocol::port> port_;

  driver_msgs::msg::Actuator msg_buffer_;

  using ProtoActuator = protolink__driver_msgs__Actuator::driver_msgs__Actuator;
  std::shared_ptr<protolink::serial_protocol::Publisher<ProtoActuator>> protolink_publisher_;

  rclcpp::Subscription<driver_msgs::msg::Thruster>::SharedPtr thr_sub_;
  rclcpp::Subscription<driver_msgs::msg::LED>::SharedPtr led_sub_;

  void thruster_callback(driver_msgs::msg::Thruster::SharedPtr _msg);
  void led_callback(driver_msgs::msg::LED::SharedPtr _msg);
};

}  // namespace actuator_rp2040_driver

#endif
