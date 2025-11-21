/**
 * @file button_battery_state.cpp
 * @brief button battery level check
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details ボタン電池残量に関するデータを取得し，トピックに流す
 **************************************************************/

#include "sensor_board_driver/button_battery_state.hpp"

using namespace std::chrono_literals;

namespace sensor_board_driver
{

ButtonBatterState::ButtonBatterState() : Node("button_battery_state")
{
  sub_port = this->declare_parameter("sub_port", 9000);
  timeout_ms = this->declare_parameter("timeout_ms", 1000);

  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ms * 1e6);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::ButtonBatteryState>("button_battery_state", qos);

  protolink_subscriber_ =
    std::make_shared<protolink::udp_protocol::Subscriber<ProtoButtonBatteryState>>(
      io_context_, sub_port, [this](const auto & _msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        timeout_->reset(this->get_clock()->now());

        driver_msgs::msg::ButtonBatteryState msg = convert(_msg);
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "battery_link";

        pub_->publish(std::move(msg));
        RCLCPP_DEBUG(
          this->get_logger(), "Publish ---> Leak: %d, RTC: %d", _msg.battery_leak(),
          _msg.battery_rtc());
      });

  RCLCPP_INFO(this->get_logger(), "ButtonBatteryState Listening on Port: %d", sub_port);

  timer_ = create_wall_timer(100ms, std::bind(&ButtonBatterState::_check_timeout, this));
}

void ButtonBatterState::_check_timeout()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // If timeout_ms is 0, timeout don't check
  if (timeout_ms == 0) return;

  if (timeout_->check(this->get_clock()->now())) {
    auto msg = std::make_unique<driver_msgs::msg::ButtonBatteryState>();
    msg->header.stamp = this->get_clock()->now();
    msg->status.id = common_msgs::msg::Status::ERROR;

    pub_->publish(std::move(msg));
    RCLCPP_ERROR(
      this->get_logger(), "ButtonBatteryState timeout: %lu [ns]", timeout_->get_elapsed_time());
  } else {
    RCLCPP_WARN(this->get_logger(), "Faild to get button battery state data");
    return;
  }
}

}  // namespace sensor_board_driver

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<sensor_board_driver::ButtonBatterState>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
