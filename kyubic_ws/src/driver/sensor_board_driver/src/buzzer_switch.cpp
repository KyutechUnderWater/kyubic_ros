/**
 * @file buzzer_switch.cpp
 * @brief buzzer on/off control
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details ブザーの制御
 **********************/

#include "sensor_board_driver/buzzer_switch.hpp"

namespace sensor_board_driver
{

BuzzerSwitch::BuzzerSwitch() : Node("buzzer_switch")
{
  mcu_ip_addr = this->declare_parameter("mcu_ip_addr", "192.168.9.5");
  mcu_port = this->declare_parameter("mcu_port", 9000);
  this_port = this->declare_parameter("this_port", 9000);

  protolink_publisher_ = std::make_shared<protolink::udp_protocol::Publisher<ProtoBuzzerSwitch>>(
    io_context_, mcu_ip_addr, mcu_port, this_port, this->get_logger());

  sub_ = this->create_subscription<driver_msgs::msg::BuzzerSwitch>(
    "buzzer_switch", 10, [this](const driver_msgs::msg::BuzzerSwitch::SharedPtr msg) {
      protolink_publisher_->send(convert(*msg));

      RCLCPP_DEBUG(
        this->get_logger(), "Protolink Publish ---> Buzzer: %d, Stop: %d", msg->buzzer,
        msg->buzzer_stop);
    });

  RCLCPP_INFO(this->get_logger(), "BuzzerSwitch Connected to %s:%d", mcu_ip_addr.c_str(), mcu_port);
}

}  // namespace sensor_board_driver

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<sensor_board_driver::BuzzerSwitch>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
