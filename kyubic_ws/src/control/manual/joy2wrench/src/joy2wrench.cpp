#include "joy2wrench/joy2wrench.hpp"

#include <cassert>
#include <iostream>

using namespace std::chrono_literals;

namespace joy2wrench
{
Joy2WrenchStamped::Joy2WrenchStamped() : Node("joy_to_wrench_stamped")
{
  device_name = this->declare_parameter("device_name", "");
  force_x_scale = this->declare_parameter("force_x_scale", 1.0);
  force_y_scale = this->declare_parameter("force_y_scale", 1.0);
  force_z_scale = this->declare_parameter("force_z_scale", 1.0);
  torque_x_scale = this->declare_parameter("torque_x_scale", 1.0);
  torque_z_scale = this->declare_parameter("torque_z_scale", 1.0);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("robot_force", qos);
  sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&Joy2WrenchStamped::_joyCallback, this, std::placeholders::_1));
}

void Joy2WrenchStamped::_joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  auto wrench_msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
  wrench_msg->header = msg->header;

  // TODO: PS3有線追加
  if (device_name == "PLAYSTATION(R)3 Controller") {
    wrench_msg->wrench.force.x = msg->axes[1] * force_x_scale;
    wrench_msg->wrench.force.y = msg->axes[0] * -force_y_scale;
    wrench_msg->wrench.force.z = msg->axes[4] * -force_z_scale;

    if (msg->axes[2] < 1 && msg->axes[5]) {
      wrench_msg->wrench.torque.x = -(msg->axes[2] - 1) * -torque_x_scale;
    } else if (msg->axes[2] && msg->axes[5] < 1) {
      wrench_msg->wrench.torque.x = -(msg->axes[5] - 1) * torque_x_scale;
    } else {
      wrench_msg->wrench.torque.x = 0.0;
    }

    wrench_msg->wrench.torque.z = msg->axes[3] * -torque_z_scale;
  } else if (device_name == "Logicool Dual Action") {
    wrench_msg->wrench.force.x = msg->axes[1] * force_x_scale;
    wrench_msg->wrench.force.y = msg->axes[0] * -force_y_scale;
    wrench_msg->wrench.force.z = msg->axes[3] * -force_z_scale;

    if (msg->buttons[6] && !msg->buttons[7]) {
      wrench_msg->wrench.torque.x = -torque_x_scale;
    } else if (!msg->buttons[6] && msg->buttons[7]) {
      wrench_msg->wrench.torque.x = torque_x_scale;
    } else {
      wrench_msg->wrench.torque.x = 0.0;
    }

    wrench_msg->wrench.torque.z = msg->axes[2] * -torque_z_scale;
  } else {
    std::cerr << "Not found device: " << device_name << std::endl;
  }
  wrench_msg->wrench.torque.y = 0.0;

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
