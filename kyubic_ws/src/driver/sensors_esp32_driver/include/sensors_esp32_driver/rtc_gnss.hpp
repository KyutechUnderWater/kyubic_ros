/**
 * @file rtc_gnss.hpp
 * @brief Get gnss data from GNSS module for RTC
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details GNSSから緯度経度、時間、衛星数を取得し，トピックに流す
*****************************************************************/

#ifndef _RTC_GNSS_HPP
#define _RTC_GNSS_HPP

#include <cstdint>
#include <driver_msgs/msg/rtc_gnss.hpp>
#include <memory>
#include <mutex>
#include <proto_files/conversion_driver_msgs__RtcGnss.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <timer/timeout.hpp>

namespace driver::sensors_esp32_driver
{

/**
 * @brief RTC GNSS class
 */
class RtcGnss : public rclcpp::Node
{
public:
  explicit RtcGnss(const rclcpp::NodeOptions & options);

private:
  uint16_t sub_port;
  uint64_t timeout_ms;

  protolink::IoContext io_context_;
  std::shared_ptr<protolink::udp_protocol::soket> sock_;
  std::shared_ptr<timer::Timeout> timeout_;
  std::mutex mutex_;

  using ProtoRtcGnss = protolink__driver_msgs__RtcGnss::driver_msgs__RtcGnss;
  std::shared_ptr<protolink::udp_protocol::Subscriber<ProtoRtcGnss>> protolink_subscriber_;

  rclcpp::Publisher<driver_msgs::msg::RtcGnss>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void protolink_callback(const ProtoRtcGnss & _msg);
};

}  // namespace driver::sensors_esp32_driver

#endif
