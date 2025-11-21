/**
 * @file depth_driver.hpp
 * @brief depth driver
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details バッテリーに関する警告データを取得し，トピックに流す
 *********************************************/

#ifndef _BATTERY_WARNING_DRIVER_HPP
#define _BATTERY_WARNING_DRIVER_HPP

#include <cstdint>
#include <driver_msgs/msg/battery_warning.hpp>
#include <mutex>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <timer/timeout.hpp>

// ビルド時に生成される変換用ヘッダー
#include <proto_files/conversion_driver_msgs__BatteryWarning.hpp>

namespace battery_warning_driver
{

// 型エイリアス
using RosBattery = driver_msgs::msg::BatteryWarning;
using ProtoBattery = protolink__driver_msgs__BatteryWarning::driver_msgs__BatteryWarning;

// =========================================================
// Convert関数
// =========================================================

// Protolink -> ROS (受信時: Getterを使用)
inline RosBattery convert(const ProtoBattery & p_msg)
{
  RosBattery msg;
  // () をつけて関数として呼び出す
  msg.battery_leak = p_msg.battery_leak();
  msg.battery_rtc = p_msg.battery_rtc();
  return msg;
}

// ROS -> Protolink (送信時: Setterを使用)
inline ProtoBattery convert(const RosBattery & msg)
{
  ProtoBattery p_msg;
  p_msg.set_battery_leak(msg.battery_leak);
  p_msg.set_battery_rtc(msg.battery_rtc);
  return p_msg;
}
// =========================================================

/**
 * @brief Battery Warning driver class
 */
class BatteryWarningDriver : public rclcpp::Node
{
public:
  explicit BatteryWarningDriver();

private:
  boost::asio::io_context io_context_;

  // Network settings
  uint16_t sub_port;
  uint64_t timeout_ms;

  // タイムアウト管理
  std::shared_ptr<timer::Timeout> timeout_;
  std::mutex mutex_;

  // Protolink Subscriber (UDP受信)
  std::shared_ptr<protolink::udp_protocol::Subscriber<ProtoBattery>> protolink_subscriber_;

  // ROS Publisher
  rclcpp::Publisher<RosBattery>::SharedPtr pub_;

  // タイムアウト監視用タイマー
  rclcpp::TimerBase::SharedPtr timer_;

  void _check_timeout();
};

}  // namespace battery_warning_driver

#endif