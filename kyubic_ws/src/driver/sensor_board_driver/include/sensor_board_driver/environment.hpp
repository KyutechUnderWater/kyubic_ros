/**
 * @file environment.hpp
 * @brief environment monitoring
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details 環境データを取得し，トピックに流す
 *********************************************/

#ifndef _ENVIRONMENT_HPP
#define _ENVIRONMENT_HPP

#include <cstdint>
#include <driver_msgs/msg/environment.hpp>
#include <mutex>
#include <proto_files/conversion_driver_msgs__Environment.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <timer/timeout.hpp>

/**
 * @namespace sensor_board_driver
 * @brief driver for sensor_board
 */
namespace sensor_board_driver
{

/**
 * @brief Environment class
 */
class Environment : public rclcpp::Node
{
public:
  explicit Environment();

private:
  boost::asio::io_context io_context_;

  uint16_t sub_port;
  uint64_t timeout_ms;

  std::shared_ptr<timer::Timeout> timeout_;
  std::mutex mutex_;

  using ProtoEnvironment = protolink__driver_msgs__Environment::driver_msgs__Environment;
  std::shared_ptr<protolink::udp_protocol::Subscriber<ProtoEnvironment>> protolink_subscriber_;

  rclcpp::Publisher<driver_msgs::msg::Environment>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void _check_timeout();
};

}  // namespace sensor_board_driver

#endif
