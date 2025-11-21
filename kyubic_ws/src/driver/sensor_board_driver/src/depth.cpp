/**
 * @file depth.cpp
 * @brief Get depth data
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details 深度データを取得し，トピックに流す
 * reference: yusukemizoguchi on 22/06/26.
 *********************************************/

#include "sensor_board_driver/depth.hpp"

using namespace std::chrono_literals;

namespace sensor_board_driver
{

Depth::Depth() : Node("depth")
{
  sub_port = this->declare_parameter("sub_port", 9000);
  timeout_ms = this->declare_parameter("timeout_ms", 1000);

  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ms * 1e6);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::Depth>("depth", qos);

  protolink_subscriber_ = std::make_shared<protolink::udp_protocol::Subscriber<protoDepth>>(
    io_context_, sub_port, [this](const auto & _msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      timeout_->reset(this->get_clock()->now());

      driver_msgs::msg::Depth msg = convert(_msg);
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "depth";

      pub_->publish(std::move(msg));
      RCLCPP_INFO(
        this->get_logger(), "Publish ---> depth: %.2f  temp: %.2f", msg.depth, msg.temperature);
    });
  RCLCPP_INFO(this->get_logger(), "Connected Port: %d", sub_port);

  timer_ = create_wall_timer(100ms, std::bind(&Depth::_check_timeout, this));
}

void Depth::_check_timeout()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Error if timeout, otherwise warning and wait
  if (timeout_->check(this->get_clock()->now())) {
    auto msg = std::make_unique<driver_msgs::msg::Depth>();
    msg->status.id = common_msgs::msg::Status::ERROR;

    pub_->publish(std::move(msg));
    RCLCPP_ERROR(
      this->get_logger(), "Depth driver timeout: %lu [ns]", timeout_->get_elapsed_time());
  } else {
    RCLCPP_WARN(this->get_logger(), "Faild to get depth data");
    return;
  }
}

}  // namespace sensor_board_driver

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<sensor_board_driver::Depth>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
