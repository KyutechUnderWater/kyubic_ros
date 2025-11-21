/**
 * @file rtc_time_driver.cpp
 * @brief RTC Time driver implementation
 ** @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details RTCモジュールから日時を取得し，トピックに流す
 * 
 **/

#include "rtc_time_driver/rtc_time_driver.hpp"

#include <iostream>

using namespace std::chrono_literals;

namespace rtc_time_driver
{

RtcTimeDriver::RtcTimeDriver() : Node("rtc_time_driver")
{
  // パラメータ設定 (ポート番号は重複しないように設定: 例 115204)
  sub_port = this->declare_parameter("sub_port", 115204);
  timeout_ns = this->declare_parameter("timeout", 0);  // 0なら無効

  // タイムアウト管理
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ns);

  // パブリッシャ
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<RosRtcTime>("rtc_time", qos);

  // Protolink受信設定
  protolink_subscriber_ = std::make_shared<protolink::udp_protocol::Subscriber<ProtoRtcTime>>(
    io_context_, sub_port, [this](const auto & _msg) {
      std::lock_guard<std::mutex> lock(mutex_);

      // タイムアウトリセット
      timeout_->reset(this->get_clock()->now());

      // 変換処理
      RosRtcTime msg = convert(_msg);

      // ヘッダー付与
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "time_link";
      msg.status.id = common_msgs::msg::Status::NORMAL;

      // パブリッシュ
      pub_->publish(msg);

      // ログ出力 (YYYY/MM/DD HH:MM:SS 形式)
      RCLCPP_INFO(
        this->get_logger(), "Subscribe & Publish ---> %04d/%02d/%02d %02d:%02d:%02d", msg.year,
        msg.month, msg.day, msg.hour, msg.minute, msg.second);
    });

  RCLCPP_INFO(this->get_logger(), "RTC Time Driver Connected Port: %d", sub_port);

  // タイムアウト監視
  timer_ = create_wall_timer(10ms, std::bind(&RtcTimeDriver::_check_timeout, this));
}

void RtcTimeDriver::_check_timeout()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (timeout_ns == 0) return;

  if (timeout_->check(this->get_clock()->now())) {
    auto msg = std::make_unique<RosRtcTime>();

    msg->status.id = common_msgs::msg::Status::ERROR;
    msg->header.stamp = this->get_clock()->now();

    pub_->publish(std::move(msg));

    RCLCPP_ERROR(
      this->get_logger(),
      //   *this->get_clock(),
      //   2000,
      "RTC Time driver timeout: %lu [ns]", timeout_->get_elapsed_time());
  }
}

}  // namespace rtc_time_driver

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<rtc_time_driver::RtcTimeDriver>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}