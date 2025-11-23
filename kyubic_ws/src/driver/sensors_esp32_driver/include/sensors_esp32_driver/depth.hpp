/**
 * @file depth.hpp
 * @brief Get depth data
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details 深度データを取得し，トピックに流す
 *********************************************/

#ifndef _DEPTH_HPP
#define _DEPTH_HPP

#include <cstdint>
#include <driver_msgs/msg/depth.hpp>
#include <proto_files/conversion_driver_msgs__BoolStamped.hpp>
#include <proto_files/conversion_driver_msgs__Depth.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <timer/timeout.hpp>

/**
 * @namespace depth_driver
 * @brief For depth driver
 */
namespace sensors_esp32_driver
{

/**
 * @brief Depth class
 */
class Depth : public rclcpp::Node
{
public:
  /**
   * @brief Bar30 & ROS-topic setting
   * @details Protolink and Define a callback function to flow the acquired data
   */
  explicit Depth(const rclcpp::NodeOptions & options);

private:
  // Network settings
  std::string mcu_ip_addr;
  uint16_t mcu_port;
  uint16_t this_port;
  uint64_t timeout_ms;

  protolink::IoContext io_context_;
  std::shared_ptr<protolink::udp_protocol::soket> sock_;
  std::shared_ptr<timer::Timeout> timeout_;
  std::mutex mutex_;

  using protoDepth = protolink__driver_msgs__Depth::driver_msgs__Depth;
  using protoBoolStamped = protolink__driver_msgs__BoolStamped::driver_msgs__BoolStamped;
  std::shared_ptr<protolink::udp_protocol::Publisher<protoBoolStamped>> protolink_publisher_;
  std::shared_ptr<protolink::udp_protocol::Subscriber<protoDepth>> protolink_subscriber_;

  rclcpp::Publisher<driver_msgs::msg::Depth>::SharedPtr pub_;
  rclcpp::Subscription<driver_msgs::msg::BoolStamped>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void protolink_callback(const protoDepth & _msg);
  void ros_callback(const driver_msgs::msg::BoolStamped & _msg);
};

}  // namespace sensors_esp32_driver

#endif
