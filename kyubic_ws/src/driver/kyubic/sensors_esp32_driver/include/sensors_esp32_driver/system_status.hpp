/**
 * @file system_status.hpp
 * @brief Get system status
 * @author R.Ohnishi
 * @date 2025/12/04
 *
 * @details Jetson, DVL等の電源供給状態の確認
 ********************************************/

#ifndef _SYSTEM_STATUS_HPP
#define _SYSTEM_STATUS_HPP

#include <cstdint>
#include <driver_msgs/msg/system_status.hpp>
#include <mutex>
#include <proto_files/conversion_driver_msgs__SystemStatus.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <timer/timeout.hpp>

namespace driver::sensors_esp32_driver
{
/**
 * @brief System Status class
 */
class SystemStatus : public rclcpp::Node
{
public:
  explicit SystemStatus(const rclcpp::NodeOptions & options);

private:
  // Network settings
  uint16_t sub_port;
  uint64_t timeout_ms;

  protolink::IoContext io_context_;
  std::shared_ptr<boost::asio::ip::udp::socket> sock_;
  std::shared_ptr<timer::Timeout> timeout_;
  std::mutex mutex_;

  using ProtoSystemStatus = protolink__driver_msgs__SystemStatus::driver_msgs__SystemStatus;
  std::shared_ptr<protolink::udp_protocol::Subscriber<ProtoSystemStatus>> protolink_subscriber_;

  rclcpp::Publisher<driver_msgs::msg::SystemStatus>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void protolink_callback(const ProtoSystemStatus & _msg);
};

}  // namespace driver::sensors_esp32_driver

#endif
