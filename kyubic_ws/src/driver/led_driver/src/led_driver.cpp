/**
 * @file led_driver.cpp
 * @brief LED driver
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details LEDのduty比を受け取り，LED制御マイコンに命令する
 */

#include "led_driver/led_driver.hpp"

namespace led_driver
{

LEDDriver::LEDDriver() : rclcpp::Node("led_driver")
{
  mcu_ip_addr = this->declare_parameter("mcu_ip_addr", "192.168.9.5");
  mcu_port = this->declare_parameter("mcu_port", 9000);
  this_port = this->declare_parameter("this_port", 9000);

  protolink_publisher_ = std::make_shared<protolink::udp_protocol::Publisher<protoLED>>(
    io_context_, mcu_ip_addr, mcu_port, this_port, this->get_logger());
  sub_ = this->create_subscription<driver_msgs::msg::LED>(
    "led_duty", 10, [this](const driver_msgs::msg::LED::SharedPtr _msg) {
      protolink_publisher_->send(convert(*_msg));
      RCLCPP_INFO(
        this->get_logger(), "Protolink Publish  --->  led1: %d  led2: %d\n", _msg->led1,
        _msg->led2);
    });
  RCLCPP_INFO(this->get_logger(), "Connected %s:%d", mcu_ip_addr.c_str(), mcu_port);
}

}  // namespace led_driver

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<led_driver::LEDDriver>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
