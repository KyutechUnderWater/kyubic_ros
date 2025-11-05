#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <joy_common_msgs/msg/joy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace joy2wrench
{

class Joy2WrenchStamped : public rclcpp::Node
{
public:
  explicit Joy2WrenchStamped(const rclcpp::NodeOptions & options);

private:
  std::string device_name;
  double force_x_scale, force_y_scale, force_z_scale;
  double torque_x_scale, torque_z_scale;

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::Subscription<joy_common_msgs::msg::Joy>::SharedPtr sub_;

  void _joyCallback(const joy_common_msgs::msg::Joy::SharedPtr msg);
};

}  // namespace joy2wrench
