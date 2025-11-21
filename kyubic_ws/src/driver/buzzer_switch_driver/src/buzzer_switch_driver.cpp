/**
 * @file buzzer_switch_driver.cpp
 * @brief Buzzer Switch driver implementation
 */

#include "buzzer_switch_driver/buzzer_switch_driver.hpp"

// ビルドエラー対策のため、ここでも標準ライブラリとROSライブラリを明示的にインクルードします
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace buzzer_switch_driver
{

BuzzerSwitchDriver::BuzzerSwitchDriver() : Node("buzzer_switch_driver")
{
  // パラメータ設定
  mcu_ip_addr = this->declare_parameter("mcu_ip_addr", "192.168.9.5");
  mcu_port = this->declare_parameter("mcu_port", 9000);
  this_port = this->declare_parameter("this_port", 9005);

  // Protolink Publisher初期化
  protolink_publisher_ = std::make_shared<protolink::udp_protocol::Publisher<ProtoBuzzer>>(
    io_context_, mcu_ip_addr, mcu_port, this_port, this->get_logger());

  // ROS Subscriber初期化
  sub_ = this->create_subscription<RosBuzzer>(
    "buzzer_switch", 10, [this](const RosBuzzer::SharedPtr msg) {
      // 変換して送信
      protolink_publisher_->send(convert(*msg));

      // ログ出力
      RCLCPP_INFO(
        this->get_logger(), "Protolink Publish ---> Buzzer: %s, Stop: %s",
        msg->buzzer ? "ON" : "OFF", msg->buzzer_stop ? "ON" : "OFF");
    });

  RCLCPP_INFO(
    this->get_logger(), "BuzzerSwitch Driver Connected to %s:%d", mcu_ip_addr.c_str(), mcu_port);
}

}  // namespace buzzer_switch_driver

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<buzzer_switch_driver::BuzzerSwitchDriver>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}