#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <string>

namespace joy2wrench
{

class Joy2WrenchStamped : public rclcpp::Node
{
public:
  explicit Joy2WrenchStamped();

private:
  std::string device_name;
  double force_scale;
  double torque_scale;

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;

  void _joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
};

}  // namespace joy2wrench
