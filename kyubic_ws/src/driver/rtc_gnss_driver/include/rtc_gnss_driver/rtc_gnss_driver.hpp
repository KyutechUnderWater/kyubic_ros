/**
* @file rtc_gnss_driver.hpp
* @brief rtc_gnss_driver 
* @author R.Ohnishi
* @date 2025/11/21
*
* @details GNSSから緯度経度、時間、衛星数を取得し，トピックに流す
*********************************************/

#ifndef _RTC_GNSS_DRIVER_HPP
#define _RTC_GNSS_DRIVER_HPP

#include <common_msgs/msg/status.hpp>
#include <cstdint>
#include <driver_msgs/msg/rtc_gnss.hpp>
#include <memory>
#include <mutex>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <timer/timeout.hpp>
#include <utility>

// Protolink生成ヘッダー
#include <proto_files/conversion_driver_msgs__RtcGnss.hpp>

namespace rtc_gnss_driver
{

// 型エイリアス
using RosRtcGnss = driver_msgs::msg::RtcGnss;
using ProtoRtcGnss = protolink__driver_msgs__RtcGnss::driver_msgs__RtcGnss;

// =========================================================
// Convert関数
// =========================================================

// Protolink -> ROS (受信時)
inline RosRtcGnss convert(const ProtoRtcGnss & p_msg)
{
  RosRtcGnss msg;
  msg.latitude = p_msg.latitude();
  msg.longitude = p_msg.longitude();
  msg.altitude = p_msg.altitude();
  msg.satellites = p_msg.satellites();
  return msg;
}

// ROS -> Protolink (送信時)
inline ProtoRtcGnss convert(const RosRtcGnss & msg)
{
  ProtoRtcGnss p_msg;
  p_msg.set_latitude(msg.latitude);
  p_msg.set_longitude(msg.longitude);
  p_msg.set_altitude(msg.altitude);
  p_msg.set_satellites(msg.satellites);
  return p_msg;
}
// =========================================================

/**
 * @brief RTC GNSS driver class
 */
class RtcGnssDriver : public rclcpp::Node
{
public:
  explicit RtcGnssDriver();

private:
  boost::asio::io_context io_context_;

  // ネットワーク設定
  uint16_t sub_port;
  uint64_t timeout_ns;

  std::shared_ptr<timer::Timeout> timeout_;
  std::mutex mutex_;

  // Protolink Subscriber
  std::shared_ptr<protolink::udp_protocol::Subscriber<ProtoRtcGnss>> protolink_subscriber_;

  // ROS Publisher
  rclcpp::Publisher<RosRtcGnss>::SharedPtr pub_;

  // タイマー
  rclcpp::TimerBase::SharedPtr timer_;

  void _check_timeout();
};

}  // namespace rtc_gnss_driver

#endif