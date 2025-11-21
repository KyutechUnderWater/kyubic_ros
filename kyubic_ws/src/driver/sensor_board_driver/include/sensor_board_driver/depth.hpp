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
#include <proto_files/conversion_driver_msgs__Depth.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <timer/timeout.hpp>

/**
 * @namespace depth_driver
 * @brief For depth driver
 */
namespace sensor_board_driver
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
  explicit Depth();

private:
  boost::asio::io_context io_context_;

  // Network settings
  uint16_t sub_port;
  uint64_t timeout_ms;

  std::shared_ptr<timer::Timeout> timeout_;

  std::mutex mutex_;

  using protoDepth = protolink__driver_msgs__Depth::driver_msgs__Depth;
  std::shared_ptr<protolink::udp_protocol::Subscriber<protoDepth>> protolink_subscriber_;

  rclcpp::Publisher<driver_msgs::msg::Depth>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void _check_timeout();
};

}  // namespace sensor_board_driver

#endif
