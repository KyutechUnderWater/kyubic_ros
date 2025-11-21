/**
 * @file leak.cpp
 * @brief leak sensor driver
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details leak sensorの情報を取得して，Topicに流す
 **************************************************/

#include "sensor_board_driver/leak.hpp"

using namespace std::chrono_literals;

namespace sensor_board_driver
{

Leak::Leak() : rclcpp::Node("leak")
{
  this_port = this->declare_parameter("this_port", 9000);
  timeout_ms = this->declare_parameter("timeout", 1000);

  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ms * 1e6);

  pub_ = this->create_publisher<driver_msgs::msg::BoolStamped>("leak", 10);

  protolink_subscriber_ = std::make_shared<protolink::udp_protocol::Subscriber<protoBoolStamped>>(
    io_context_, this_port, [this](const auto & _msg) {
      pub_->publish(convert(_msg));
      RCLCPP_DEBUG(this->get_logger(), "Protolink Publish  --->  data: %d\n", _msg.data());
    });

  RCLCPP_INFO(this->get_logger(), "Connected port: %d", this_port);

  timer_ = create_wall_timer(100ms, std::bind(&Leak::_check_timeout, this));
}

void Leak::_check_timeout()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // If timeout_ms is 0, timeout don't check
  if (timeout_ms == 0) return;

  if (timeout_->check(this->get_clock()->now())) {
    auto msg = std::make_unique<driver_msgs::msg::BoolStamped>();
    msg->header.stamp = this->get_clock()->now();
    msg->status.id = common_msgs::msg::Status::ERROR;

    pub_->publish(std::move(msg));
    RCLCPP_ERROR(this->get_logger(), "Environment timeout: %lu [ns]", timeout_->get_elapsed_time());
  } else {
    RCLCPP_WARN(this->get_logger(), "Faild to get leak data");
    return;
  }
}

}  // namespace sensor_board_driver

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<sensor_board_driver::Leak>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
