#include "joy2wrench/joy2wrench.hpp"

using namespace std::chrono_literals;

namespace joy2wrench
{
Joy2WrenchStamped::Joy2WrenchStamped(const rclcpp::NodeOptions & options)
: Node("joy_to_wrench_stamped", options)
{
  force_x_scale = this->declare_parameter("force_x_scale", 1.0);
  force_y_scale = this->declare_parameter("force_y_scale", 1.0);
  force_z_scale = this->declare_parameter("force_z_scale", 1.0);
  torque_x_scale = this->declare_parameter("torque_x_scale", 1.0);
  torque_z_scale = this->declare_parameter("torque_z_scale", 1.0);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("robot_force", qos);
  sub_ = create_subscription<joy_common_msgs::msg::Joy>(
    "joy_common", 10, std::bind(&Joy2WrenchStamped::_joyCallback, this, std::placeholders::_1));
}

void Joy2WrenchStamped::_joyCallback(const joy_common_msgs::msg::Joy::SharedPtr msg)
{
  auto wrench_msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
  wrench_msg->header = msg->header;

  wrench_msg->wrench.force.x = msg->stick.ly * force_x_scale;
  wrench_msg->wrench.force.y = msg->stick.lx * -force_y_scale;
  wrench_msg->wrench.force.z = msg->stick.ry * -force_z_scale;

  if (msg->button.l2 < 1 && msg->button.r2) {
    wrench_msg->wrench.torque.x = -(msg->button.l2 - 1) * -torque_x_scale;
  } else if (msg->button.l2 && msg->button.r2 < 1) {
    wrench_msg->wrench.torque.x = -(msg->button.r2 - 1) * torque_x_scale;
  } else {
    wrench_msg->wrench.torque.x = 0.0;
  }

  wrench_msg->wrench.torque.y = 0.0;
  wrench_msg->wrench.torque.z = msg->stick.rx * -torque_z_scale;

  pub_->publish(std::move(wrench_msg));
}
}  // namespace joy2wrench

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(joy2wrench::Joy2WrenchStamped)
