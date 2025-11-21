/**
 * @file rtc_time_driver.hpp
 * @brief RTC Time driver
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details RTCモジュールから日時を取得し，トピックに流す
*********************************************/

#ifndef _RTC_TIME_DRIVER_HPP
#define _RTC_TIME_DRIVER_HPP

#include <common_msgs/msg/status.hpp>
#include <cstdint>
#include <driver_msgs/msg/rtc_time.hpp>
#include <memory>
#include <mutex>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <timer/timeout.hpp>
#include <utility>

// Protolink生成ヘッダー
#include <proto_files/conversion_driver_msgs__RtcTime.hpp>

namespace rtc_time_driver
{

// 型エイリアス
using RosRtcTime = driver_msgs::msg::RtcTime;
using ProtoRtcTime = protolink__driver_msgs__RtcTime::driver_msgs__RtcTime;

// =========================================================
// Convert関数 (ROS <-> Protolink)
// =========================================================

// Protolink -> ROS (受信時)
inline RosRtcTime convert(const ProtoRtcTime & p_msg)
{
  RosRtcTime msg;
  msg.year = p_msg.year();
  msg.month = p_msg.month();
  msg.day = p_msg.day();
  msg.hour = p_msg.hour();
  msg.minute = p_msg.minute();
  msg.second = p_msg.second();
  return msg;
}

// ROS -> Protolink (送信時)
inline ProtoRtcTime convert(const RosRtcTime & msg)
{
  ProtoRtcTime p_msg;
  p_msg.set_year(msg.year);
  p_msg.set_month(msg.month);
  p_msg.set_day(msg.day);
  p_msg.set_hour(msg.hour);
  p_msg.set_minute(msg.minute);
  p_msg.set_second(msg.second);
  return p_msg;
}
// =========================================================

/**
 * @brief RTC Time driver class
 */
class RtcTimeDriver : public rclcpp::Node
{
public:
  explicit RtcTimeDriver();

private:
  boost::asio::io_context io_context_;

  // ネットワーク設定
  uint16_t sub_port;
  uint64_t timeout_ns;

  std::shared_ptr<timer::Timeout> timeout_;
  std::mutex mutex_;

  // Protolink Subscriber
  std::shared_ptr<protolink::udp_protocol::Subscriber<ProtoRtcTime>> protolink_subscriber_;

  // ROS Publisher
  rclcpp::Publisher<RosRtcTime>::SharedPtr pub_;

  // タイマー
  rclcpp::TimerBase::SharedPtr timer_;

  void _check_timeout();
};

}  // namespace rtc_time_driver

#endif