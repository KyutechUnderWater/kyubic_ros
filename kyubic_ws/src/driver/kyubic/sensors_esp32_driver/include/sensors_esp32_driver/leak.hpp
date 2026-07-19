/**
 * @file leak.hpp
 * @brief leak sensor driver
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details leak sensorの情報を取得して，Topicに流す
 **************************************************/

#ifndef _LEAK_HPP
#define _LEAK_HPP

#include <driver_msgs/msg/bool_stamped.h>

#include <memory>
#include <proto_files/conversion_driver_msgs__BoolStamped.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <timer/timeout.hpp>

/**
 * @namespace driver::sensors_esp32_driver
 * @brief driver for sensors_esp32
 */
namespace driver::sensors_esp32_driver
{

/**
 * @brief Leak class
 */
class Leak : public rclcpp::Node
{
public:
  /**
   * @brief Protolink & ROS-topic settng
   * @details Protolink setting and Define a publisher
   */
  explicit Leak(const rclcpp::NodeOptions & options);

private:
  uint16_t sub_port;  // of the computer executing this code
  uint64_t timeout_ms;

  protolink::IoContext io_context_;
  std::shared_ptr<protolink::udp_protocol::soket> sock_;
  std::shared_ptr<timer::Timeout> timeout_;
  std::mutex mutex_;

  using ProtoBoolStamped = protolink__driver_msgs__BoolStamped::driver_msgs__BoolStamped;
  std::shared_ptr<protolink::udp_protocol::Subscriber<ProtoBoolStamped>> protolink_subscriber_;

  rclcpp::Publisher<driver_msgs::msg::BoolStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void protolink_callback(const ProtoBoolStamped & _msg);
};

}  // namespace driver::sensors_esp32_driver

#endif  // !_LEAK_HPP
