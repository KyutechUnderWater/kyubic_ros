/**
 * @file led_driver.hpp
 * @brief LED driver
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details LEDのduty比を受け取り，LED制御マイコンに命令する
 */

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>

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
  std::string portname;
  int baudrate;

  float duty;

  std::shared_ptr<serial::Serial> serial_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;

  void _led_callback(const std_msgs::msg::Float32 msg);
};

}  // namespace led_driver
