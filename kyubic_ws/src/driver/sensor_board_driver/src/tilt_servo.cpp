/**
 * @file tilt_servo.cpp
 * @brief servo driver for camera tilt
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details カメラのチルト用サーボモータの信号をSensor基板に送信
 **************************************************************/

#include "sensor_board_driver/tilt_servo.hpp"

namespace sensor_board_driver
{

TiltServo::TiltServo() : rclcpp::Node("tilt_servo")
{
  mcu_ip_addr = this->declare_parameter("mcu_ip_addr", "192.168.9.5");
  mcu_port = this->declare_parameter("mcu_port", 9000);
  this_port = this->declare_parameter("this_port", 9000);

  protolink_publisher_ = std::make_shared<protolink::udp_protocol::Publisher<protoInt32Stamped>>(
    io_context_, mcu_ip_addr, mcu_port, this_port, this->get_logger());

  sub_ = this->create_subscription<driver_msgs::msg::Int32Stamped>(
    "tilt_servo", 10, [this](const driver_msgs::msg::Int32Stamped::SharedPtr _msg) {
      protolink_publisher_->send(convert(*_msg));
      RCLCPP_DEBUG(this->get_logger(), "Protolink Publish  --->  data: %d\n", _msg->data);
    });

  RCLCPP_INFO(this->get_logger(), "Connected %s:%d", mcu_ip_addr.c_str(), mcu_port);
}

}  // namespace sensor_board_driver

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<sensor_board_driver::TiltServo>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
