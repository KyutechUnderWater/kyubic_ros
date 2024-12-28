#include "dvl_driver/dvl_driver.hpp"

using namespace std::chrono_literals;

namespace dvl_driver
{

DVLDriver::DVLDriver() : Node("dvl_driver")
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<kyubic_interface_msgs::msg::PathFinder>("path_finder", qos);
  timer_ = create_wall_timer(100ms, std::bind(&DVLDriver::update, this));
}

kyubic_interface_msgs::msg::PathFinder::UniquePtr create_msg(std::shared_ptr<DVLData> dvl_data_)
{
  kyubic_interface_msgs::msg::PathFinder::UniquePtr pathfinder_;

  pathfinder_->header.frame_id = "/pathfinder";

  pathfinder_->velocity.x = dvl_data_->x_vel_bottom;
  pathfinder_->velocity.y = dvl_data_->y_vel_bottom;
  pathfinder_->velocity.z = dvl_data_->z_vel_bottom;
  pathfinder_->velocity_error = dvl_data_->e_vel_bottom;

  pathfinder_->depth = 0.0;
  pathfinder_->altitude = dvl_data_->altitude;

  return pathfinder_;
}

void DVLDriver::update()
{
  listener_->listen();
  auto msg = create_msg(listener_->get_dvl_data());
  pub_->publish(std::move(msg));
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
