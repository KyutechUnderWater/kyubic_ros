/**
 * @file led_driver.hpp
 * @brief LED driver
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details LEDのduty比を受け取り，LED制御マイコンに命令する
 */

#include <driver_msgs/msg/led.hpp>
#include <proto_files/conversion_driver_msgs__LED.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @namespace led_driver
 * @brief For led driver
 */
namespace led_driver
{

/**
 * @brief LED driver class
 */
class LEDDriver : public rclcpp::Node
{
public:
  /**
   * @brief Serial & ROS-topic settng
   * @details Serial setting and Define a subscriber
   */
  explicit LEDDriver();

private:
  boost::asio::io_context io_context_;
  std::string mcu_ip_addr;  // of microcontroller, etc.
  uint16_t mcu_port;        // same as above
  uint16_t this_port;       // of the computer executing this code

  float duty;

  using protoLED = protolink__driver_msgs__LED::driver_msgs__LED;
  std::shared_ptr<protolink::udp_protocol::Publisher<protoLED>> protolink_publisher_;

  rclcpp::Subscription<driver_msgs::msg::LED>::SharedPtr sub_;
};

}  // namespace led_driver
