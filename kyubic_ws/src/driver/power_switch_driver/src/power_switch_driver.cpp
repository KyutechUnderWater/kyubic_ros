/**
 * @file power_switch_driver.cpp
 * @brief Power Switch driver implementation
 */

#include "power_switch_driver/power_switch_driver.hpp"

namespace power_switch_driver
{

PowerSwitchDriver::PowerSwitchDriver() : rclcpp::Node("power_switch_driver")
{
  // パラメータ設定 (必要に応じてデフォルト値を変更してください)
  mcu_ip_addr = this->declare_parameter("mcu_ip_addr", "192.168.9.5");
  mcu_port = this->declare_parameter("mcu_port", 9000);
  this_port = this->declare_parameter("this_port", 9002);  // ポート重複を避けるため適当な値を設定

  // Protolink Publisherの初期化
  protolink_publisher_ = std::make_shared<protolink::udp_protocol::Publisher<ProtoPowerSwitch>>(
    io_context_, mcu_ip_addr, mcu_port, this_port, this->get_logger());

  // ROS Subscriberの初期化
  sub_ = this->create_subscription<RosPowerSwitch>(
    "power_switch_cmd", 10, [this](const RosPowerSwitch::SharedPtr msg) {
      // データを変換してUDP送信
      protolink_publisher_->send(convert(*msg));

      // ログ出力 (boolをintにキャストして表示 1=ON, 0=OFF)
      RCLCPP_INFO(
        this->get_logger(), "PowerSwitch CMD ---> Jetson:%d DVL:%d COM:%d EX1:%d EX2:%d ACT:%d",
        (int)msg->jetson, (int)msg->dvl, (int)msg->com, (int)msg->ex1, (int)msg->ex2,
        (int)msg->actuator);
    });

  RCLCPP_INFO(
    this->get_logger(), "PowerSwitch Driver Connected to %s:%d", mcu_ip_addr.c_str(), mcu_port);
}

}  // namespace power_switch_driver

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<power_switch_driver::PowerSwitchDriver>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}