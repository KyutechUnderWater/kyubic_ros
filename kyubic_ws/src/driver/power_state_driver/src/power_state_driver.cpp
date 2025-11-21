/**
* @file power_state_driver.cpp
* @brief power_state_driver
* @author R.Ohnishi
* @date 2025/11/21
*
* @details それぞれの電源の状態を取得し，トピックに流す
*********************************************/

#include "power_state_driver/power_state_driver.hpp"

#include <iostream>

using namespace std::chrono_literals;

namespace power_state_driver
{

PowerStateDriver::PowerStateDriver() : Node("power_state_driver")
{
  // パラメータ設定
  sub_port = this->declare_parameter("sub_port", 9004);
  timeout_ns = this->declare_parameter("timeout", 2000000000);  // 2秒

  // タイムアウト管理
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ns);

  // パブリッシャ
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<RosPowerState>("power_state", qos);

  // Protolink受信設定
  protolink_subscriber_ = std::make_shared<protolink::udp_protocol::Subscriber<ProtoPowerState>>(
    io_context_, sub_port, [this](const auto & _msg) {
      std::lock_guard<std::mutex> lock(mutex_);

      // タイムアウトリセット
      timeout_->reset(this->get_clock()->now());

      // 変換処理
      RosPowerState msg = convert(_msg);

      // ヘッダー付与
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "power_link";
      msg.status.id = common_msgs::msg::Status::NORMAL;

      // パブリッシュ
      pub_->publish(msg);

      // ログ出力
      RCLCPP_INFO(
        this->get_logger(), "PowerState Recv ---> Jetson:%s Act:%s Relay:%s USB:%s",
        msg.jetson ? "ON" : "OFF", msg.actuator_power ? "ON" : "OFF",
        msg.logic_relay ? "ON" : "OFF", msg.usb_power ? "ON" : "OFF");
    });

  RCLCPP_INFO(this->get_logger(), "PowerState Driver Listening on Port: %d", sub_port);

  // タイムアウト監視タイマー
  timer_ = create_wall_timer(10ms, std::bind(&PowerStateDriver::_check_timeout, this));
}

void PowerStateDriver::_check_timeout()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // タイムアウト設定が0の場合は無効
  if (timeout_ns == 0) return;

  // タイムアウト判定
  if (timeout_->check(this->get_clock()->now())) {
    auto msg = std::make_unique<RosPowerState>();

    msg->status.id = common_msgs::msg::Status::ERROR;
    msg->header.stamp = this->get_clock()->now();

    pub_->publish(std::move(msg));

    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(),
      5000,  // 5秒間隔で警告
      "PowerState driver timeout: %lu [ns]", timeout_->get_elapsed_time());
  }
}

}  // namespace power_state_driver

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<power_state_driver::PowerStateDriver>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}