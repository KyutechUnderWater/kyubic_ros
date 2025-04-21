#include "joy2wrench/joy2wrench.hpp"

using namespace std::chrono_literals;

namespace joy2wrench
{
Joy2WrenchStamped::Joy2WrenchStamped() : Node("joy_to_wrench_stamped")
{
  force_scale = this->declare_parameter("force_scale", 1.0);
  torque_scale = this->declare_parameter("torque_scale", 1.0);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("robot_force", qos);
  sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&Joy2WrenchStamped::_joyCallback, this, std::placeholders::_1));
}

void Joy2WrenchStamped::_joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  auto wrench_msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
  wrench_msg->header = msg->header;

  wrench_msg->wrench.force.x = msg->axes[1] * force_scale;
  wrench_msg->wrench.force.y = msg->axes[0] * force_scale * -1.0;
  wrench_msg->wrench.force.z = msg->axes[3] * force_scale;

  wrench_msg->wrench.torque.x = 0.0;
  wrench_msg->wrench.torque.y = 0.0;
  wrench_msg->wrench.torque.z = msg->axes[2] * torque_scale * -1.0;

  pub_->publish(std::move(wrench_msg));
}
}  // namespace joy2wrench

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<joy2wrench::Joy2WrenchStamped>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
