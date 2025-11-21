/**
* @file rtc_gnss_driver.cpp
* @brief rtc_gnss_driver
* @author R.Ohnishi
* @date 2025/11/21
*
* @details GNSSから緯度経度、時間、衛星数を取得し，トピックに流す
*********************************************/

#include "rtc_gnss_driver/rtc_gnss_driver.hpp"

#include <iostream>

using namespace std::chrono_literals;

namespace rtc_gnss_driver
{

RtcGnssDriver::RtcGnssDriver() : Node("rtc_gnss_driver")
{
  // パラメータ設定 (ポート番号は重複しないように設定してください: 例 115203)
  sub_port = this->declare_parameter("sub_port", 115203);
  timeout_ns = this->declare_parameter("timeout", 0);  // 0ならタイムアウト無効

  // タイムアウト管理
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ns);

  // パブリッシャ
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<RosRtcGnss>("rtc_gnss", qos);

  // Protolink受信設定
  protolink_subscriber_ = std::make_shared<protolink::udp_protocol::Subscriber<ProtoRtcGnss>>(
    io_context_, sub_port, [this](const auto & _msg) {
      std::lock_guard<std::mutex> lock(mutex_);

      // タイムアウトリセット
      timeout_->reset(this->get_clock()->now());

      // 変換処理
      RosRtcGnss msg = convert(_msg);

      // ヘッダー付与
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "gnss_link";
      msg.status.id = common_msgs::msg::Status::NORMAL;

      // パブリッシュ
      pub_->publish(msg);

      RCLCPP_INFO(
        this->get_logger(), "Subscribe & Publish ---> Lat: %.6f, Lon: %.6f, Alt: %.2f, Sat: %d",
        msg.latitude, msg.longitude, msg.altitude, msg.satellites);
    });

  RCLCPP_INFO(this->get_logger(), "RTC GNSS Driver Connected Port: %d", sub_port);

  // タイムアウト監視
  timer_ = create_wall_timer(10ms, std::bind(&RtcGnssDriver::_check_timeout, this));
}

void RtcGnssDriver::_check_timeout()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // タイムアウト設定が0の場合は何もしない
  if (timeout_ns == 0) return;

  // タイムアウト判定
  if (timeout_->check(this->get_clock()->now())) {
    auto msg = std::make_unique<RosRtcGnss>();

    msg->status.id = common_msgs::msg::Status::ERROR;
    msg->header.stamp = this->get_clock()->now();

    pub_->publish(std::move(msg));

    RCLCPP_ERROR(
      this->get_logger(),
      //   *this->get_clock(),
      //   2000, // 2秒間隔
      "RTC GNSS driver timeout: %lu [ns]", timeout_->get_elapsed_time());
  }
}

}  // namespace rtc_gnss_driver

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<rtc_gnss_driver::RtcGnssDriver>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}