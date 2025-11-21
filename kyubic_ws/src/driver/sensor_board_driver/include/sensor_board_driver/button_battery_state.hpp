/**
 * @file button_battery_state.hpp
 * @brief button battery level check
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details ボタン電池残量に関するデータを取得し，トピックに流す
 **************************************************************/

#ifndef _BUTTON_BATTERY_STATUS_HPP
#define _BUTTON_BATTERY_STATUS_HPP

#include <cstdint>
#include <driver_msgs/msg/button_battery_state.hpp>
#include <mutex>
#include <proto_files/conversion_driver_msgs__ButtonBatteryState.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <timer/timeout.hpp>

namespace sensor_board_driver
{
/**
 * @brief Button Battery State class
 */
class ButtonBatterState : public rclcpp::Node
{
public:
  explicit ButtonBatterState();

private:
  boost::asio::io_context io_context_;

  // Network settings
  uint16_t sub_port;
  uint64_t timeout_ms;

  std::shared_ptr<timer::Timeout> timeout_;
  std::mutex mutex_;

  using ProtoButtonBatteryState =
    protolink__driver_msgs__ButtonBatteryState::driver_msgs__ButtonBatteryState;
  std::shared_ptr<protolink::udp_protocol::Subscriber<ProtoButtonBatteryState>>
    protolink_subscriber_;

  rclcpp::Publisher<driver_msgs::msg::ButtonBatteryState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void _check_timeout();
};

}  // namespace sensor_board_driver

#endif
