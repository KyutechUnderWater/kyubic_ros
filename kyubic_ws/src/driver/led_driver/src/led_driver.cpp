/**
 * @file led_driver.cpp
 * @brief LED driver
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details LEDのduty比を受け取り，LED制御マイコンに命令する
 */

#include "led_driver/led_driver.hpp"

namespace led_driver
{

LEDDriver::LEDDriver() : rclcpp::Node("led_driver")
{
  portname = this->declare_parameter("serial_port", "/dev/ttyACM0");
  baudrate = this->declare_parameter("serial_speed", 115200);

  serial_ = std::make_shared<serial::Serial>(portname.c_str(), baudrate);
  RCLCPP_INFO(this->get_logger(), "Connected %s", portname.c_str());

  sub_ = create_subscription<std_msgs::msg::Float32>(
    "led_duty", 10, std::bind(&LEDDriver::_led_callback, this, std::placeholders::_1));
}

void LEDDriver::_led_callback(const std_msgs::msg::Float32 msg)
{
  duty = msg.data;

  if (duty < 0 || 1 < duty)
    RCLCPP_ERROR(this->get_logger(), "Out of range: duty is %f", duty);
  else {
    uint8_t buf = uint8_t(duty * 100);
    serial_->write(&buf, sizeof(buf));
    RCLCPP_INFO(this->get_logger(), "Send: %d", buf);
  }
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
