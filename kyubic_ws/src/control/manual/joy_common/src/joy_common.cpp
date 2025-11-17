/**
 * @file joy_common.cpp
 * @brief joystick library
 * @author R.Ohnishi
 * @date 2025/11/05
 *
 * @details joystickのデータを流す (PS3基準)
 ******************************************/

#include "joy_common/joy_common.hpp"

using namespace std::chrono_literals;

namespace joy_common
{
JoyCommon::JoyCommon(const rclcpp::NodeOptions & options) : Node("joy_common", options)
{
  device_name = this->declare_parameter("device_name", "");

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<joy_common_msgs::msg::Joy>("joy_common", qos);
  sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&JoyCommon::_joyCallback, this, std::placeholders::_1));
}

void JoyCommon::_joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  auto joy_msg = std::make_unique<joy_common_msgs::msg::Joy>();
  joy_msg->header = msg->header;

  if (
    device_name == "PLAYSTATION(R)3 Controller" ||
    device_name == "Sony PLAYSTATION(R)3 Controller") {
    joy_msg->stick.lx = msg->axes[0];
    joy_msg->stick.ly = msg->axes[1];
    joy_msg->stick.rx = msg->axes[3];
    joy_msg->stick.ry = msg->axes[4];

    joy_msg->button.cross = msg->buttons[0];
    joy_msg->button.circle = msg->buttons[1];
    joy_msg->button.triangle = msg->buttons[2];
    joy_msg->button.square = msg->buttons[3];
    joy_msg->button.l1 = msg->buttons[4];
    joy_msg->button.r1 = msg->buttons[5];
    joy_msg->button.l2 = msg->axes[2];
    joy_msg->button.r2 = msg->axes[5];
    joy_msg->button.select = msg->buttons[8];
    joy_msg->button.start = msg->buttons[9];
    joy_msg->button.ps = msg->buttons[10];
    joy_msg->button.l_stick = msg->buttons[11];
    joy_msg->button.r_stick = msg->buttons[12];
    joy_msg->button.up = msg->buttons[13];
    joy_msg->button.down = msg->buttons[14];
    joy_msg->button.left = msg->buttons[15];
    joy_msg->button.right = msg->buttons[16];
  } else if (device_name == "Logicool Dual Action") {
    joy_msg->stick.lx = msg->axes[0];
    joy_msg->stick.ly = msg->axes[1];
    joy_msg->stick.rx = msg->axes[2];
    joy_msg->stick.ry = msg->axes[3];

    joy_msg->button.cross = msg->buttons[1];
    joy_msg->button.circle = msg->buttons[2];
    joy_msg->button.triangle = msg->buttons[3];
    joy_msg->button.square = msg->buttons[0];
    joy_msg->button.l1 = msg->buttons[4];
    joy_msg->button.r1 = msg->buttons[5];
    joy_msg->button.select = msg->buttons[8];
    joy_msg->button.start = msg->buttons[9];
    joy_msg->button.ps = false;
    joy_msg->button.l_stick = msg->buttons[10];
    joy_msg->button.r_stick = msg->buttons[11];

    if (msg->buttons[6])
      joy_msg->button.l2 = -1;
    else
      joy_msg->button.l2 = 1;

    if (msg->buttons[7])
      joy_msg->button.r2 = -1;
    else
      joy_msg->button.r2 = 1;

    if (msg->axes[5] == 1)
      joy_msg->button.up = true;
    else if (msg->axes[5] == -1)
      joy_msg->button.down = true;
    else {
      joy_msg->button.up = false;
      joy_msg->button.down = false;
    }

    if (msg->axes[4] == 1)
      joy_msg->button.left = true;
    else if (msg->axes[4] == -1)
      joy_msg->button.right = true;
    else {
      joy_msg->button.left = false;
      joy_msg->button.right = false;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Not found device: %s", device_name.c_str());
  }

  pub_->publish(std::move(joy_msg));
}
}  // namespace joy_common

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(joy_common::JoyCommon)
