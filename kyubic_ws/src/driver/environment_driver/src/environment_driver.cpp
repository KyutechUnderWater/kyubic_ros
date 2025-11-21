/**
 * @file depth_driver.cpp
 * @brief depth driver
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details 環境データを取得し，トピックに流す
 *********************************************/

#include "environment_driver/environment_driver.hpp"

#include <iostream>

using namespace std::chrono_literals;

namespace environment_driver
{

EnvironmentDriver::EnvironmentDriver() : Node("environment_driver")
{
  // パラメータ設定 (デフォルト値)
  sub_port = this->declare_parameter("sub_port", 115200);
  timeout_ns = this->declare_parameter("timeout", 2000000000);  // 2秒 (ns)

  // タイムアウト管理
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ns);

  // パブリッシャ
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<RosEnv>("environment", qos);

  // Protolink受信設定
  protolink_subscriber_ = std::make_shared<protolink::udp_protocol::Subscriber<ProtoEnv>>(
    io_context_, sub_port, [this](const auto & _msg) {
      std::lock_guard<std::mutex> lock(mutex_);

      // データ受信によりタイムアウトをリセット
      timeout_->reset(this->get_clock()->now());

      // 変換処理
      RosEnv msg = convert(_msg);

      // ヘッダー付与
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "environment_link";
      msg.status.id = common_msgs::msg::Status::NORMAL;

      // パブリッシュ
      pub_->publish(msg);

      RCLCPP_INFO(
        this->get_logger(), "Subscribe & Publish ---> Temp: %.2f, Hum: %.2f, Press: %.2f",
        msg.temperature, msg.humidity, msg.pressure);
    });

  RCLCPP_INFO(this->get_logger(), "Environment Driver Listening on Port: %d", sub_port);

  // タイムアウト監視タイマー (10ms間隔)
  timer_ = create_wall_timer(10ms, std::bind(&EnvironmentDriver::_check_timeout, this));
}

void EnvironmentDriver::_check_timeout()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // タイムアウト設定が0の場合は無効
  if (timeout_ns == 0) return;

  // タイムアウト判定
  if (timeout_->check(this->get_clock()->now())) {
    auto msg = std::make_unique<RosEnv>();

    msg->status.id = common_msgs::msg::Status::ERROR;
    msg->header.stamp = this->get_clock()->now();

    pub_->publish(std::move(msg));

    RCLCPP_ERROR(
      this->get_logger(),
      //   *this->get_clock(),
      //   1000, // 1秒に1回ログ出力
      "Environment driver timeout: %lu [ns]", timeout_->get_elapsed_time());
  }
}

}  // namespace environment_driver

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<environment_driver::EnvironmentDriver>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}