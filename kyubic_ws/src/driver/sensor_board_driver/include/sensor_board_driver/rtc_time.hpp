/**
 * @file rtc_time.hpp
 * @brief Get RTC Time
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details RTCモジュールから日時を取得し，トピックに流す
********************************************************/

#ifndef _RTC_TIME_HPP
#define _RTC_TIME_HPP

#include <cstdint>
#include <driver_msgs/msg/rtc_time.hpp>
#include <mutex>
#include <proto_files/conversion_driver_msgs__RtcTime.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <timer/timeout.hpp>

namespace sensor_board_driver
{

/**
 * @brief RTC Time class
 */
class RtcTime : public rclcpp::Node
{
public:
  explicit RtcTime();

private:
  boost::asio::io_context io_context_;

  uint16_t sub_port;
  uint64_t timeout_ms;

  std::shared_ptr<timer::Timeout> timeout_;
  std::mutex mutex_;

  using ProtoRtcTime = protolink__driver_msgs__RtcTime::driver_msgs__RtcTime;
  std::shared_ptr<protolink::udp_protocol::Subscriber<ProtoRtcTime>> protolink_subscriber_;

  rclcpp::Publisher<driver_msgs::msg::RtcTime>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void _check_timeout();
};

}  // namespace sensor_board_driver

#endif
