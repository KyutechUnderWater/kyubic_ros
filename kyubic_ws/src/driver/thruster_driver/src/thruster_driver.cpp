/**
 * @file thruster_driver.cpp
 * @brief Thruster driver
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details スラスターの出力値を受け取り，制御マイコンに命令する
 */

#include "thruster_driver/thruster_driver.hpp"

namespace thruster_driver
{

ThrusterDriver::ThrusterDriver() : rclcpp::Node("thruster_driver")
{
  // パラメータの宣言（デフォルト値は環境に合わせて変更してください）
  mcu_ip_addr = this->declare_parameter("mcu_ip_addr", "192.168.9.5");
  mcu_port = this->declare_parameter("mcu_port", 9000);
  this_port = this->declare_parameter("this_port", 9000);  // 受信ポート

  // Protolink Publisherの初期化
  protolink_publisher_ = std::make_shared<protolink::udp_protocol::Publisher<ProtoThruster>>(
    io_context_, mcu_ip_addr, mcu_port, this_port, this->get_logger());

  // ROS Subscriberの初期化
  sub_ = this->create_subscription<RosThruster>(
    "thruster_duty", 10, [this](const RosThruster::SharedPtr _msg) {
      // データを変換して送信
      protolink_publisher_->send(convert(*_msg));

      // ログ出力 (6軸分)
      RCLCPP_INFO(
        this->get_logger(), "Protolink Publish ---> T1:%d T2:%d T3:%d T4:%d T5:%d T6:%d",
        _msg->thruster1, _msg->thruster2, _msg->thruster3, _msg->thruster4, _msg->thruster5,
        _msg->thruster6);
    });

  RCLCPP_INFO(
    this->get_logger(), "Thruster Driver Connected to %s:%d", mcu_ip_addr.c_str(), mcu_port);
}

}  // namespace thruster_driver

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<thruster_driver::ThrusterDriver>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}