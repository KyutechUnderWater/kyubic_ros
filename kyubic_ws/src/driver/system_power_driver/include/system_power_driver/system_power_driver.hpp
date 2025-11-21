/**
 * @file system_power_driver.hpp
 * @brief SystemPower driver
 */

#ifndef _SYSTEM_POWER_DRIVER_HPP
#define _SYSTEM_POWER_DRIVER_HPP

#include <common_msgs/msg/status.hpp>
#include <cstdint>
#include <driver_msgs/msg/system_power.hpp>
#include <memory>
#include <mutex>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <timer/timeout.hpp>
#include <utility>

// Protolink生成ヘッダー
#include <proto_files/conversion_driver_msgs__SystemPower.hpp>

namespace system_power_driver
{

// 型エイリアス
using RosSystemPower = driver_msgs::msg::SystemPower;
using ProtoSystemPower = protolink__driver_msgs__SystemPower::driver_msgs__SystemPower;

// =========================================================
// Convert関数
// =========================================================

// Protolink -> ROS (受信時)
inline RosSystemPower convert(const ProtoSystemPower & p_msg)
{
  RosSystemPower msg;
  msg.log_voltage = p_msg.log_voltage();
  msg.log_current = p_msg.log_current();
  msg.log_power = p_msg.log_power();
  msg.act_voltage = p_msg.act_voltage();
  msg.act_current = p_msg.act_current();
  msg.act_power = p_msg.act_power();
  msg.log_temp = p_msg.log_temp();
  msg.act_temp = p_msg.act_temp();
  return msg;
}

// ROS -> Protolink (送信時)
inline ProtoSystemPower convert(const RosSystemPower & msg)
{
  ProtoSystemPower p_msg;
  p_msg.set_log_voltage(msg.log_voltage);
  p_msg.set_log_current(msg.log_current);
  p_msg.set_log_power(msg.log_power);
  p_msg.set_act_voltage(msg.act_voltage);
  p_msg.set_act_current(msg.act_current);
  p_msg.set_act_power(msg.act_power);
  p_msg.set_log_temp(msg.log_temp);
  p_msg.set_act_temp(msg.act_temp);
  return p_msg;
}
// =========================================================

/**
 * @brief SystemPower driver class
 */
class SystemPowerDriver : public rclcpp::Node
{
public:
  explicit SystemPowerDriver();

private:
  boost::asio::io_context io_context_;

  // ネットワーク設定
  uint16_t sub_port;
  uint64_t timeout_ns;

  std::shared_ptr<timer::Timeout> timeout_;
  std::mutex mutex_;

  // Protolink Subscriber
  std::shared_ptr<protolink::udp_protocol::Subscriber<ProtoSystemPower>> protolink_subscriber_;

  // ROS Publisher
  rclcpp::Publisher<RosSystemPower>::SharedPtr pub_;

  // タイマー
  rclcpp::TimerBase::SharedPtr timer_;

  void _check_timeout();
};

}  // namespace system_power_driver

#endif