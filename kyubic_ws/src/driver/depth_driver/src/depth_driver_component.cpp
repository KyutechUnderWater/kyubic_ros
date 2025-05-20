/**
 * @file depth_driver.cpp
 * @brief depth driver
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details 深度データを取得し，トピックに流す\n\n
 * reference: yusukemizoguchi on 22/06/26.
 *********************************************/

#include "depth_driver/depth_driver_component.hpp"

using namespace std::chrono_literals;

namespace depth_driver
{

DepthDriver::DepthDriver(const rclcpp::NodeOptions & options) : Node("depth", options)
{
  portname = this->declare_parameter("serial_port", "/dev/ttyACM0");
  baudrate = this->declare_parameter("serial_speed", 115200);

  bar30_ = std::make_shared<Bar30>(portname.c_str(), baudrate);
  RCLCPP_INFO(this->get_logger(), "Connected %s", portname.c_str());

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::Depth>("depth", qos);
  timer_ = create_wall_timer(100ms, std::bind(&DepthDriver::_update, this));
}

void DepthDriver::_update()
{
  if (bar30_->update()) {
    auto msg = std::make_unique<driver_msgs::msg::Depth>();
    msg->pos_z = bar30_->get_data();

    RCLCPP_INFO(this->get_logger(), "%f", msg->pos_z);
    pub_->publish(std::move(msg));
  } else {
    RCLCPP_WARN(this->get_logger(), "Faild to get depth data");
  }
}

}  // namespace depth_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depth_driver::DepthDriver)

// int main(int argc, char ** argv)
// {
//   setvbuf(stdout, NULL, _IONBF, BUFSIZ);
//   rclcpp::init(argc, argv);
//
//   try {
//     auto node = std::make_shared<depth_driver::DepthDriver>();
//     rclcpp::spin(node);
//   } catch (std::exception & e) {
//     std::cout << e.what() << std::endl;
//   }
//
//   rclcpp::shutdown();
//   return 0;
// }
