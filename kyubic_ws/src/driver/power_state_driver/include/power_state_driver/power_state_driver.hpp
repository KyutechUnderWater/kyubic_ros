/**
* @file power_state_driver.hpp
* @brief  power_state_driver
* @author R.Ohnishi
* @date 2025/11/21
*
* @details それぞれの電源の状態を取得し，トピックに流す
*********************************************/

#ifndef _POWER_STATE_DRIVER_HPP
#define _POWER_STATE_DRIVER_HPP

// 標準ライブラリ
#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>

// ROS関連
#include <common_msgs/msg/status.hpp>
#include <driver_msgs/msg/power_state.hpp>
#include <rclcpp/rclcpp.hpp>

// Protolink関連
#include <protolink/client.hpp>
#include <timer/timeout.hpp>

// 自動生成ヘッダー
#include <proto_files/conversion_driver_msgs__PowerState.hpp>

namespace power_state_driver
{

// 型エイリアス
using RosPowerState = driver_msgs::msg::PowerState;
using ProtoPowerState = protolink__driver_msgs__PowerState::driver_msgs__PowerState;

// =========================================================
// Convert関数
// =========================================================

// Protolink -> ROS (受信時: Getter)
inline RosPowerState convert(const ProtoPowerState & p_msg)
{
  RosPowerState msg;
  msg.jetson = p_msg.jetson();
  msg.actuator_power = p_msg.actuator_power();
  msg.logic_relay = p_msg.logic_relay();
  msg.usb_power = p_msg.usb_power();
  return msg;
}

// ROS -> Protolink (送信時: Setter)
inline ProtoPowerState convert(const RosPowerState & msg)
{
  ProtoPowerState p_msg;
  p_msg.set_jetson(msg.jetson);
  p_msg.set_actuator_power(msg.actuator_power);
  p_msg.set_logic_relay(msg.logic_relay);
  p_msg.set_usb_power(msg.usb_power);
  return p_msg;
}
// =========================================================

/**
 * @brief PowerState driver class
 */
class PowerStateDriver : public rclcpp::Node
{
public:
  explicit PowerStateDriver();

private:
  boost::asio::io_context io_context_;

  // ネットワーク設定
  uint16_t sub_port;
  uint64_t timeout_ns;

  std::shared_ptr<timer::Timeout> timeout_;
  std::mutex mutex_;

  // Protolink Subscriber
  std::shared_ptr<protolink::udp_protocol::Subscriber<ProtoPowerState>> protolink_subscriber_;

  // ROS Publisher
  rclcpp::Publisher<RosPowerState>::SharedPtr pub_;

  // タイマー
  rclcpp::TimerBase::SharedPtr timer_;

  void _check_timeout();
};

}  // namespace power_state_driver

#endif