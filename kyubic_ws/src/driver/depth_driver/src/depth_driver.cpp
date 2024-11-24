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

using namespace std::chrono_literals;

namespace depth_driver
{

DepthDriver::DepthDriver() : Node("depth")
{
  portname = this->declare_parameter("serial_port", "/dev/ttyACM0");
  baudrate = this->declare_parameter("serial_speed", 115200);

  bar30_ = std::make_shared<Bar30>(portname.c_str(), baudrate);
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::Depth>("depth", qos);
  timer_ = create_wall_timer(100ms, std::bind(&DepthDriver::update, this));
}

void DepthDriver::update()
{
  if (bar30_->update()) {
    auto msg = std::make_unique<driver_msgs::msg::Depth>();
    msg->depth = bar30_->get_depth_data();

    RCLCPP_INFO(this->get_logger(), "depth: %f", msg->depth);
    pub_->publish(std::move(msg));
  } else {
    RCLCPP_WARN(this->get_logger(), "Faild to get depth data");
  }
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
