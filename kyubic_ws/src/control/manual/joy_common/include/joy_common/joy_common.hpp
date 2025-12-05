/**
 * @file joy_common.hpp
 * @brief joystick library
 * @author R.Ohnishi
 * @date 2025/11/05
 *
 * @details joystickのデータを流す
 *********************************/

#ifndef _JOY_COMMON_HPP
#define _JOY_COMMON_HPP

#include <joy_common_msgs/msg/buttons.hpp>
#include <joy_common_msgs/msg/joy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>

namespace joy_common
{

using ButtonValue = std::variant<bool, double>;
using ButtonMap =
  std::map<std::string, std::function<ButtonValue(const joy_common_msgs::msg::Buttons &)>>;

ButtonMap get_button_map();

class JoyCommon : public rclcpp::Node
{
public:
  explicit JoyCommon(const rclcpp::NodeOptions & options);

private:
  std::string device_name;
  double force_x_scale, force_y_scale, force_z_scale;
  double torque_x_scale, torque_z_scale;

  rclcpp::Publisher<joy_common_msgs::msg::Joy>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;

  void _joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
};

}  // namespace joy_common

#endif  // !_JOY_COMMON_HPP
