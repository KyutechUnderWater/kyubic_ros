/**
 * @file depth_driver.cpp
 * @brief depth driver
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details 深度データを取得し，トピックに流す\n\n
 * reference: yusukemizoguchi on 22/06/26.
 *********************************************/

#include "depth_driver/depth_driver.hpp"

#include "driver_msgs/msg/depth.hpp"

using namespace std::chrono_literals;

namespace depth_driver
{

DepthDriver::DepthDriver() : Node("depth")
{
  portname = this->declare_parameter("serial_port", "/dev/ttyACM0");
  baudrate = this->declare_parameter("serial_speed", 115200);
  timeout = this->declare_parameter("timeout", 0);
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout);

  bar30_ = std::make_shared<Bar30>(portname.c_str(), baudrate);
  RCLCPP_INFO(this->get_logger(), "Connected %s", portname.c_str());

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::Depth>("depth", qos);
  timer_ = create_wall_timer(10ms, std::bind(&DepthDriver::_update, this));
}

void DepthDriver::_update()
{
  auto msg = std::make_unique<driver_msgs::msg::Depth>();

  if (bar30_->update()) {
    timeout_->reset(this->get_clock()->now());

    // Data acquisition
    auto data = bar30_->get_data();

    // Prepare message
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = "base_link";
    msg->status = driver_msgs::msg::Depth::STATUS_NORMAL;
    msg->depth = data.depth;

    RCLCPP_INFO(this->get_logger(), "%f", msg->depth);
  } else {
    // Error if timeout, otherwise warning and wait
    if (timeout_->check(this->get_clock()->now())) {
      msg->status = driver_msgs::msg::Depth::STATUS_ERROR;

      RCLCPP_ERROR(
        this->get_logger(), "Depth driver timeout: %lu [ns]", timeout_->get_elapsed_time());
    } else {
      RCLCPP_WARN(this->get_logger(), "Faild to get depth data");
      return;
    }
  }

  // Publish
  pub_->publish(std::move(msg));
}

}  // namespace depth_driver

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<depth_driver::DepthDriver>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
