/**
 * @file depth_driver.hpp
 * @brief depth driver
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details 深度データを取得し，トピックに流す
 *********************************************/

#ifndef _DEPTH_DRIVER_HPP
#define _DEPTH_DRIVER_HPP

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
namespace depth_driver
{

/**
 * @brief Depth driver class
 */
class DepthDriver : public rclcpp::Node
{
public:
  /**
   * @brief Bar30 & ROS-topic setting
   * @details The bar30 serial setting and Define a callback function to flow the acquired data to
   * the topic at 100ms interval
   */
  explicit DepthDriver();

private:
  boost::asio::io_context io_context_;

  // Network settings
  uint16_t sub_port;
  uint64_t timeout;

  std::shared_ptr<timer::Timeout> timeout_;

  std::mutex mutex_;

  using protoDepth = protolink__driver_msgs__Depth::driver_msgs__Depth;
  std::shared_ptr<protolink::udp_protocol::Subscriber<protoDepth>> protolink_subscriber_;

  rclcpp::Publisher<driver_msgs::msg::Depth>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void _check_timeout();
};

}  // namespace depth_driver

#endif