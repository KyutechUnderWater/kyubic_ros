#include "dvl_driver/path_finder.hpp"

#include <rclcpp/rclcpp.hpp>

#include <kyubic_interface_msgs/msg/path_finder.hpp>

namespace dvl_driver
{

class DVLDriver : public rclcpp::Node
{
public:
  explicit DVLDriver();

private:
  std::shared_ptr<DVLSender> sender_;
  std::shared_ptr<DVLListener> listener_;
  rclcpp::Publisher<kyubic_interface_msgs::msg::PathFinder>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void update();
};

}  // namespace dvl_driver
