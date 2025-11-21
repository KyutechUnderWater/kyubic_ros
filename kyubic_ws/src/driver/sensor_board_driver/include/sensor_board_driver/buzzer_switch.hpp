/**
 * @file buzzer_switch.hpp
 * @brief buzzer on/off control
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details ブザーの制御
 **********************/

#ifndef _BUZZER_SWITCH_HPP
#define _BUZZER_SWITCH_HPP

#include <driver_msgs/msg/buzzer_switch.hpp>
#include <proto_files/conversion_driver_msgs__BuzzerSwitch.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sensor_board_driver
{

class BuzzerSwitch : public rclcpp::Node
{
public:
  explicit BuzzerSwitch();

private:
  boost::asio::io_context io_context_;
  std::string mcu_ip_addr;
  uint16_t mcu_port;
  uint16_t this_port;

  using ProtoBuzzerSwitch = protolink__driver_msgs__BuzzerSwitch::driver_msgs__BuzzerSwitch;
  std::shared_ptr<protolink::udp_protocol::Publisher<ProtoBuzzerSwitch>> protolink_publisher_;
  rclcpp::Subscription<driver_msgs::msg::BuzzerSwitch>::SharedPtr sub_;
};

}  // namespace sensor_board_driver

#endif
