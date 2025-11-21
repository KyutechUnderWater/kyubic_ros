/**
 * @file depth_driver.cpp
 * @brief depth driver
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details 環境データを取得し，トピックに流す
 *********************************************/
#ifndef _ENVIRONMENT_DRIVER_HPP
#define _ENVIRONMENT_DRIVER_HPP

// 必要な標準ライブラリ
#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>

// ROS関連
#include <common_msgs/msg/status.hpp>  // Status定数用
#include <driver_msgs/msg/environment.hpp>
#include <rclcpp/rclcpp.hpp>

// Protolink関連
#include <protolink/client.hpp>
#include <timer/timeout.hpp>

// 自動生成される変換ヘッダー
#include <proto_files/conversion_driver_msgs__Environment.hpp>

namespace environment_driver
{

// 型エイリアス
using RosEnv = driver_msgs::msg::Environment;
using ProtoEnv = protolink__driver_msgs__Environment::driver_msgs__Environment;

// =========================================================
// Convert関数 (ROS <-> Protolink)
// =========================================================

// Protolink -> ROS (受信時)
inline RosEnv convert(const ProtoEnv & p_msg)
{
  RosEnv msg;
  msg.temperature = p_msg.temperature();
  msg.humidity = p_msg.humidity();
  msg.pressure = p_msg.pressure();
  return msg;
}

// ROS -> Protolink (送信時)
inline ProtoEnv convert(const RosEnv & msg)
{
  ProtoEnv p_msg;
  p_msg.set_temperature(msg.temperature);
  p_msg.set_humidity(msg.humidity);
  p_msg.set_pressure(msg.pressure);
  return p_msg;
}
// =========================================================

/**
 * @brief Environment driver class
 */
class EnvironmentDriver : public rclcpp::Node
{
public:
  explicit EnvironmentDriver();

private:
  boost::asio::io_context io_context_;

  // ネットワーク設定
  uint16_t sub_port;
  uint64_t timeout_ns;  // ナノ秒単位

  std::shared_ptr<timer::Timeout> timeout_;
  std::mutex mutex_;

  // Protolink Subscriber
  std::shared_ptr<protolink::udp_protocol::Subscriber<ProtoEnv>> protolink_subscriber_;

  // ROS Publisher
  rclcpp::Publisher<RosEnv>::SharedPtr pub_;

  // タイマー
  rclcpp::TimerBase::SharedPtr timer_;

  void _check_timeout();
};

}  // namespace environment_driver

#endif