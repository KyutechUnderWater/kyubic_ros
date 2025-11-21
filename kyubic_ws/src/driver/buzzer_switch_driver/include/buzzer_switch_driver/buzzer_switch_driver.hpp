#ifndef _BUZZER_SWITCH_DRIVER_HPP
#define _BUZZER_SWITCH_DRIVER_HPP

#include <driver_msgs/msg/buzzer_switch.hpp>
#include <memory>
#include <proto_files/conversion_driver_msgs__BuzzerSwitch.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace buzzer_switch_driver
{

using RosBuzzer = driver_msgs::msg::BuzzerSwitch;
using ProtoBuzzer = protolink__driver_msgs__BuzzerSwitch::driver_msgs__BuzzerSwitch;

inline ProtoBuzzer convert(const RosBuzzer & msg)
{
  ProtoBuzzer p_msg;
  p_msg.set_buzzer(msg.buzzer);
  p_msg.set_buzzer_stop(msg.buzzer_stop);
  return p_msg;
}

class BuzzerSwitchDriver : public rclcpp::Node
{
public:
  explicit BuzzerSwitchDriver();

private:
  boost::asio::io_context io_context_;
  std::string mcu_ip_addr;
  uint16_t mcu_port;
  uint16_t this_port;

  std::shared_ptr<protolink::udp_protocol::Publisher<ProtoBuzzer>> protolink_publisher_;
  rclcpp::Subscription<RosBuzzer>::SharedPtr sub_;
};

}  // namespace buzzer_switch_driver

#endif