/**
 * @file rgb.hpp
 * @brief neopixel rgb driver
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details NeoPixelに入力するRGBの値をSensor基板に送信
 *****************************************************/

#ifndef _RGB_HPP
#define _RGB_HPP

#include <driver_msgs/msg/int32_stamped.hpp>
#include <memory>
#include <proto_files/conversion_driver_msgs__Int32Stamped.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @namespace sensors_esp32_driver
 * @brief driver for sensors_esp32
 */
namespace sensors_esp32_driver
{

/**
 * @brief RBG class
 */
class RGB : public rclcpp::Node
{
public:
  /**
   * @brief Protolink & ROS-topic settng
   * @details Protolink setting and Define a subscriber
   */
  explicit RGB(const rclcpp::NodeOptions & options);

private:
  std::string mcu_ip_addr;  // of microcontroller, etc.
  uint16_t mcu_port;        // same as above
  uint16_t this_port;       // of the computer executing this code

  protolink::IoContext io_context_;
  std::shared_ptr<protolink::udp_protocol::soket> sock_;

  using ProtoInt32Stamped = protolink__driver_msgs__Int32Stamped::driver_msgs__Int32Stamped;
  std::shared_ptr<protolink::udp_protocol::Publisher<ProtoInt32Stamped>> protolink_publisher_;

  rclcpp::Subscription<driver_msgs::msg::Int32Stamped>::SharedPtr sub_;
};

}  // namespace sensors_esp32_driver

#endif  // _RGB_HPP
