/**
 * @file environment.cpp
 * @brief environment monitoring
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details 環境データを取得し，トピックに流す
 *********************************************/

#include "sensor_board_driver/environment.hpp"

using namespace std::chrono_literals;

namespace sensor_board_driver
{

Environment::Environment() : Node("environment")
{
  sub_port = this->declare_parameter("sub_port", 9000);
  timeout_ms = this->declare_parameter("timeout", 1000);

  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ms * 1e6);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::Environment>("environment", qos);

  protolink_subscriber_ = std::make_shared<protolink::udp_protocol::Subscriber<ProtoEnvironment>>(
    io_context_, sub_port, [this](const auto & _msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      timeout_->reset(this->get_clock()->now());

      driver_msgs::msg::Environment msg = convert(_msg);
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "environment_link";

      pub_->publish(msg);
      RCLCPP_DEBUG(
        this->get_logger(), "Publish ---> Temp: %.2f, Hum: %.2f, Press: %.2f", msg.temperature,
        msg.humidity, msg.pressure);
    });

  RCLCPP_INFO(this->get_logger(), "Environment Listening on Port: %d", sub_port);

  timer_ = create_wall_timer(100ms, std::bind(&Environment::_check_timeout, this));
}

void Environment::_check_timeout()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // If timeout_ms is 0, timeout don't check
  if (timeout_ms == 0) return;

  if (timeout_->check(this->get_clock()->now())) {
    auto msg = std::make_unique<driver_msgs::msg::Environment>();
    msg->header.stamp = this->get_clock()->now();
    msg->status.id = common_msgs::msg::Status::ERROR;

    pub_->publish(std::move(msg));
    RCLCPP_ERROR(this->get_logger(), "Environment timeout: %lu [ns]", timeout_->get_elapsed_time());
  } else {
    RCLCPP_WARN(this->get_logger(), "Faild to get environment data");
    return;
  }
}

}  // namespace sensor_board_driver

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<sensor_board_driver::Environment>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
