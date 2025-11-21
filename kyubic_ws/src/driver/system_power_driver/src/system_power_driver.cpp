/**
 * @file system_power_driver.cpp
 * @brief SystemPower driver implementation
 */

#include "system_power_driver/system_power_driver.hpp"

#include <iostream>

using namespace std::chrono_literals;

namespace system_power_driver
{

SystemPowerDriver::SystemPowerDriver() : Node("system_power_driver")
{
  // パラメータ設定 (ポート番号は重複しないように設定: 例 115205)
  sub_port = this->declare_parameter("sub_port", 115205);
  timeout_ns = this->declare_parameter("timeout", 0);

  // タイムアウト管理
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ns);

  // パブリッシャ
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<RosSystemPower>("system_power", qos);

  // Protolink受信設定
  protolink_subscriber_ = std::make_shared<protolink::udp_protocol::Subscriber<ProtoSystemPower>>(
    io_context_, sub_port, [this](const auto & _msg) {
      std::lock_guard<std::mutex> lock(mutex_);

      // タイムアウトリセット
      timeout_->reset(this->get_clock()->now());

      // 変換処理
      RosSystemPower msg = convert(_msg);

      // ヘッダー付与
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "power_link";
      msg.status.id = common_msgs::msg::Status::NORMAL;

      // パブリッシュ
      pub_->publish(msg);

      RCLCPP_INFO(
        this->get_logger(), "SystemPower Recv ---> Log: %.2fV/%.2fA, Act: %.2fV/%.2fA",
        msg.log_voltage, msg.log_current, msg.act_voltage, msg.act_current);
    });

  RCLCPP_INFO(this->get_logger(), "SystemPower Driver Connected Port: %d", sub_port);

  // タイムアウト監視
  timer_ = create_wall_timer(10ms, std::bind(&SystemPowerDriver::_check_timeout, this));
}

void SystemPowerDriver::_check_timeout()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (timeout_ns == 0) return;

  if (timeout_->check(this->get_clock()->now())) {
    auto msg = std::make_unique<RosSystemPower>();

    msg->status.id = common_msgs::msg::Status::ERROR;
    msg->header.stamp = this->get_clock()->now();

    pub_->publish(std::move(msg));

    RCLCPP_ERROR(
      this->get_logger(),
      //   *this->get_clock(),
      //   2000,
      "SystemPower driver timeout: %lu [ns]", timeout_->get_elapsed_time());
  }
}

}  // namespace system_power_driver

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<system_power_driver::SystemPowerDriver>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}