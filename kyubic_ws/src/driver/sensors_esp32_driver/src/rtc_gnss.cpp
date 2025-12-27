/**
 * @file rtc_gnss.cpp
 * @brief Get gnss data from GNSS module for RTC
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details GNSSから緯度経度、時間、衛星数を取得し，トピックに流す
*****************************************************************/

#include "sensors_esp32_driver/rtc_gnss.hpp"

#include <functional>
#include <protolink/udp_protocol.hpp>

#include "sensors_esp32_driver/common/utils.hpp"

using namespace std::chrono_literals;

namespace sensors_esp32_driver
{

RtcGnss::RtcGnss(const rclcpp::NodeOptions & options) : Node("rtc_gnss", options)
{
  sub_port = this->declare_parameter("sub_port", 9000);
  timeout_ms = this->declare_parameter("timeout_ms", 1000);

  try {
    sock_ = protolink::udp_protocol::create_socket(io_context_, sub_port);
    RCLCPP_INFO(this->get_logger(), "RTC GNSS open port: %d", sub_port);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize udp publisher: %s", e.what());
    RCLCPP_ERROR(this->get_logger(), "RTC GNSS don't open port %d", sub_port);
    exit(1);
  }
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ms * 1e6);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::RtcGnss>("rtc_gnss", rclcpp::SensorDataQoS());

  protolink_subscriber_ = std::make_shared<protolink::udp_protocol::Subscriber<ProtoRtcGnss>>(
    sock_, std::bind(&RtcGnss::protolink_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(std::chrono::milliseconds(timeout_ms / 2), [this]() {
    if (timeout_ms)
      check_timeout<driver_msgs::msg::RtcGnss>(this, mutex_, timeout_, pub_, "RtcGnss");
  });
}

void RtcGnss::protolink_callback(const ProtoRtcGnss & _msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    timeout_->reset(this->get_clock()->now());
  }

  auto msg =
    std::make_unique<driver_msgs::msg::RtcGnss>(protolink__driver_msgs__RtcGnss::convert(_msg));
  msg->header.stamp = this->get_clock()->now();
  msg->header.frame_id = "gnss_link";

  pub_->publish(std::move(msg));
  RCLCPP_DEBUG(
    this->get_logger(), "Publish ---> Lat: %.6f, Lon: %.6f, Alt: %.2f, Sat: %d", _msg.latitude(),
    _msg.longitude(), _msg.altitude(), _msg.satellites());
}

}  // namespace sensors_esp32_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sensors_esp32_driver::RtcGnss)
