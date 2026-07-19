/**
* @file logic_distro_rp2040_driver.hpp
* @brief  Get voltage, current, and power. Set system state
* @author R.Ohnishi
* @date 2025/11/21
*
* @details RP2040と通信する
******************************************************/

#ifndef _LOGIC_DISTRO_RP2040_DRIVER_HPP
#define _LOGIC_DISTRO_RP2040_DRIVER_HPP

#include <cstdint>
#include <driver_msgs/msg/power_state.hpp>
#include <driver_msgs/msg/system_switch.hpp>
#include <mutex>
#include <proto_files/conversion_driver_msgs__PowerState.hpp>
#include <proto_files/conversion_driver_msgs__SystemSwitch.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <timer/timeout.hpp>

namespace driver::logic_distro_rp2040_driver
{

/**
 * @brief PowerState driver class
 */
class LogicDistroRP2040 : public rclcpp::Node
{
public:
  explicit LogicDistroRP2040(const rclcpp::NodeOptions & options);

private:
  std::string portname;
  uint32_t baudrate;
  std::string mcu_ip_addr;
  uint16_t mcu_port;
  uint16_t this_port;
  uint64_t timeout_ms;

  protolink::IoContext io_context_;
  std::shared_ptr<protolink::serial_protocol::port> port_;
  std::shared_ptr<protolink::udp_protocol::soket> sock_;
  std::shared_ptr<timer::Timeout> timeout_;
  std::mutex mutex_;

  using ProtoPowerState = protolink__driver_msgs__PowerState::driver_msgs__PowerState;
  using ProtoSystemSwitch = protolink__driver_msgs__SystemSwitch::driver_msgs__SystemSwitch;
  std::shared_ptr<protolink::udp_protocol::Publisher<ProtoPowerState>> protolink_publisher_udp_;
  std::shared_ptr<protolink::serial_protocol::Publisher<ProtoSystemSwitch>> protolink_publisher_;
  std::shared_ptr<protolink::serial_protocol::Subscriber<ProtoPowerState>> protolink_subscriber_;

  rclcpp::Publisher<driver_msgs::msg::PowerState>::SharedPtr pub_;
  rclcpp::Subscription<driver_msgs::msg::SystemSwitch>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void protolink_callback(const ProtoPowerState & _msg);
  void ros_callback(const driver_msgs::msg::SystemSwitch & _msg);

  void _check_timeout();
};

}  // namespace driver::logic_distro_rp2040_driver

#endif
