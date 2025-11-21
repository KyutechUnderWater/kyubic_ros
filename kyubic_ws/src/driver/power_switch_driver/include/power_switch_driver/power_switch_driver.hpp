/**
 * @file power_switch_driver.hpp
 * @brief Power Switch driver
 */

#ifndef _POWER_SWITCH_DRIVER_HPP
#define _POWER_SWITCH_DRIVER_HPP

#include <driver_msgs/msg/power_switch.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>

// ビルド時に生成される変換用ヘッダー
#include <proto_files/conversion_driver_msgs__PowerSwitch.hpp>

namespace power_switch_driver
{
// 型エイリアス
using RosPowerSwitch = driver_msgs::msg::PowerSwitch;
using ProtoPowerSwitch = protolink__driver_msgs__PowerSwitch::driver_msgs__PowerSwitch;

// =========================================================
// Convert関数 (ROS -> Protolink)
// =========================================================
inline ProtoPowerSwitch convert(const RosPowerSwitch & msg)
{
  ProtoPowerSwitch p_msg;

  // booleanの値をセット (Protolinkの生成コードは通常 bool をサポートします)
  p_msg.set_jetson(msg.jetson);
  p_msg.set_dvl(msg.dvl);
  p_msg.set_com(msg.com);
  p_msg.set_ex1(msg.ex1);
  p_msg.set_ex2(msg.ex2);
  p_msg.set_actuator(msg.actuator);

  return p_msg;
}
// =========================================================

/**
 * @brief Power Switch driver class
 */
class PowerSwitchDriver : public rclcpp::Node
{
public:
  explicit PowerSwitchDriver();

private:
  boost::asio::io_context io_context_;
  std::string mcu_ip_addr;
  uint16_t mcu_port;
  uint16_t this_port;

  // Protolink Publisher (UDP送信)
  std::shared_ptr<protolink::udp_protocol::Publisher<ProtoPowerSwitch>> protolink_publisher_;

  // ROS Subscriber (コマンド受信)
  rclcpp::Subscription<RosPowerSwitch>::SharedPtr sub_;
};

}  // namespace power_switch_driver

#endif