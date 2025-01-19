/**
 * @file dvl_driver.cpp
 * @brief DVL driver
 * @author R.Ohnishi
 * @date 2024/10/30
 *
 * @details DVL(Path Finder) のデータを取得して，Topicを流す
 ************************************************************/

#include "dvl_driver/dvl_driver.hpp"

using namespace std::chrono_literals;

namespace dvl_driver
{

DVLDriver::DVLDriver() : Node("dvl_driver")
{
  address = this->declare_parameter("ip_address", "0.0.0.0");
  port = this->declare_parameter("port", 8888);

  listener_ = std::make_shared<path_finder::Listener>(address.c_str(), port);
  sender_ = std::make_shared<path_finder::Sender>(address.c_str(), port);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::PathFinder>("path_finder", qos);
  timer_ = create_wall_timer(100ms, std::bind(&DVLDriver::update, this));
}

driver_msgs::msg::PathFinder::UniquePtr DVLDriver::_create_msg(
  std::shared_ptr<path_finder::Data> dvl_data_)
{
  driver_msgs::msg::PathFinder::UniquePtr pathfinder_;

  pathfinder_->header.frame_id = "/pathfinder";

  pathfinder_->velocity.x = dvl_data_->x_vel_bottom;
  pathfinder_->velocity.y = dvl_data_->y_vel_bottom;
  pathfinder_->velocity.z = dvl_data_->z_vel_bottom;
  pathfinder_->velocity_error = dvl_data_->e_vel_bottom;

  pathfinder_->depth = 0.0;
  pathfinder_->altitude = dvl_data_->altitude;

  return pathfinder_;
}

bool DVLDriver::update()
{
  if (!sender_->ping()) return false;
  if (!listener_->listen()) return false;

  auto msg = _create_msg(listener_->get_dvl_data());
  pub_->publish(std::move(msg));
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
