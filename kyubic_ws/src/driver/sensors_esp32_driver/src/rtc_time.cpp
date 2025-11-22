/**
 * @file rtc_time.cpp
 * @brief Get RTC Time
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details RTCモジュールから日時を取得し，トピックに流す
********************************************************/

#include "sensors_esp32_driver/rtc_time.hpp"

#include <functional>
#include <protolink/udp_protocol.hpp>

#include "sensors_esp32_driver/common/utils.hpp"

using namespace std::chrono_literals;

namespace sensors_esp32_driver
{

RtcTime::RtcTime(const rclcpp::NodeOptions & options) : Node("rtc_time", options)
{
  sub_port = this->declare_parameter("sub_port", 9000);
  timeout_ms = this->declare_parameter("timeout", 1000);

  try {
    sock_ = protolink::udp_protocol::create_socket(io_context_, sub_port);
    RCLCPP_INFO(this->get_logger(), "RTC Time open port: %d", sub_port);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize udp publisher: %s", e.what());
    RCLCPP_ERROR(this->get_logger(), "RTC Time don't open port %d", sub_port);
    exit(1);
  }
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ms * 1e6);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::RtcTime>("rtc_time", qos);

  protolink_subscriber_ = std::make_shared<protolink::udp_protocol::Subscriber<ProtoRtcTime>>(
    sock_, std::bind(&RtcTime::protolink_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(100ms, [this]() {
    if (timeout_ms)
      check_timeout<driver_msgs::msg::RtcTime>(this, mutex_, timeout_, pub_, "RtcTime");
  });
}

void RtcTime::protolink_callback(const ProtoRtcTime & _msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  timeout_->reset(this->get_clock()->now());

  auto msg =
    std::make_unique<driver_msgs::msg::RtcTime>(protolink__driver_msgs__RtcTime::convert(_msg));
  msg->header.stamp = this->get_clock()->now();
  msg->header.frame_id = "time_link";

  pub_->publish(std::move(msg));
  RCLCPP_DEBUG(
    this->get_logger(), "Publish ---> %04d/%02d/%02d %02d:%02d:%02d", _msg.year(), _msg.month(),
    _msg.day(), _msg.hour(), _msg.minute(), _msg.second());
}

}  // namespace sensors_esp32_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sensors_esp32_driver::RtcTime)
