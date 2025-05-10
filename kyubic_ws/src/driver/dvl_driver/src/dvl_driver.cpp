/**
 * @file dvl_driver.cpp
 * @brief DVL driver
 * @author R.Ohnishi
 * @date 2024/10/30
 *
 * @details DVL(Path Finder) のデータを取得して，Topicを流す
 ************************************************************/

#include "dvl_driver/dvl_driver.hpp"

#include <iostream>

using namespace std::chrono_literals;

namespace dvl_driver
{

DVLDriver::DVLDriver() : Node("dvl_driver")
{
  address = this->declare_parameter("ip_address", "0.0.0.0");
  listener_port = this->declare_parameter("listener_port", 8888);
  sender_port = this->declare_parameter("sender_port", 8889);

  listener_ = std::make_shared<path_finder::Listener>(address.c_str(), listener_port, 500);
  sender_ = std::make_shared<path_finder::Sender>(address.c_str(), sender_port, 500);
  RCLCPP_INFO(this->get_logger(), "DVL connection successful");

  if (!setup()) exit(1);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::DVL>("dvl", qos);
  timer_ = create_wall_timer(100ms, std::bind(&DVLDriver::update, this));
}

bool DVLDriver::setup()
{
  if (!sender_->break_cmd()) {
    RCLCPP_ERROR(this->get_logger(), "Failded to send the break command.");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Send break command.");
  return true;
}

driver_msgs::msg::DVL::UniquePtr DVLDriver::_create_msg(
  std::shared_ptr<path_finder::Data> dvl_data_)
{
  auto msgs = std::make_unique<driver_msgs::msg::DVL>();

  msgs->header.frame_id = "/pathfinder";

  msgs->velocity.x = dvl_data_->x_vel_bottom;
  msgs->velocity.y = dvl_data_->y_vel_bottom;
  msgs->velocity.z = dvl_data_->z_vel_bottom;
  msgs->velocity_error = dvl_data_->e_vel_bottom;

  msgs->depth = 0.0;
  msgs->altitude = dvl_data_->altitude;

  return msgs;
}

bool DVLDriver::update()
{
  if (!sender_->ping_cmd()) {
    RCLCPP_ERROR(this->get_logger(), "ping faild");
    return false;
  }

  if (!listener_->listen()) {
    RCLCPP_ERROR(this->get_logger(), "listen faild");
    return false;
  }

  auto msg = _create_msg(listener_->get_dvl_data());
  pub_->publish(std::move(msg));

  RCLCPP_INFO(this->get_logger(), "Update DVL data");
  return true;
}

}  // namespace dvl_driver

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<dvl_driver::DVLDriver>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
