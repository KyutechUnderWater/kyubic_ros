/**
 * @file thruster_driver.hpp
 * @brief Thruster driver
 * @author K.Fujimoto
 * @date 2025/11/21
 *
 * @details スラスターの出力値を受け取り，制御マイコンに命令する
 */

#ifndef _THRUSTER_DRIVER_HPP
#define _THRUSTER_DRIVER_HPP

#include <driver_msgs/msg/thruster.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>

// ビルド時に生成される変換用ヘッダー (ファイル名は環境に依存しますが、一般的にこの形式です)
#include <proto_files/conversion_driver_msgs__Thruster.hpp>

namespace thruster_driver
{

// 型エイリアス
using RosThruster = driver_msgs::msg::Thruster;
using ProtoThruster = protolink__driver_msgs__Thruster::driver_msgs__Thruster;

// =========================================================
// Convert関数の実装 (ROS -> Protolink)
// =========================================================
inline ProtoThruster convert(const RosThruster & msg)
{
  ProtoThruster p_msg;

  // HeaderやStatusは必要に応じてコピーしてください
  // p_msg.mutable_header()->set_frame_id(msg.header.frame_id);

  // 6つのスラスター値をセット
  p_msg.set_thruster1(msg.thruster1);
  p_msg.set_thruster2(msg.thruster2);
  p_msg.set_thruster3(msg.thruster3);
  p_msg.set_thruster4(msg.thruster4);
  p_msg.set_thruster5(msg.thruster5);
  p_msg.set_thruster6(msg.thruster6);

  return p_msg;
}
// =========================================================

/**
 * @brief Thruster driver class
 */
class ThrusterDriver : public rclcpp::Node
{
public:
  /**
   * @brief Serial & ROS-topic settng
   * @details Serial setting and Define a subscriber
   */
  explicit ThrusterDriver();

private:
  boost::asio::io_context io_context_;
  std::string mcu_ip_addr;  // of microcontroller
  uint16_t mcu_port;        // same as above
  uint16_t this_port;       // of this computer

  // Protolink Publisher
  std::shared_ptr<protolink::udp_protocol::Publisher<ProtoThruster>> protolink_publisher_;

  // ROS Subscriber
  rclcpp::Subscription<RosThruster>::SharedPtr sub_;
};

}  // namespace thruster_driver

#endif