/**
 * @file joy_common.hpp
 * @brief joystick library
 * @author R.Ohnishi
 * @date 2025/11/05
 *
 * @details joystickのデータを流す
 *********************************/

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

ButtonMap get_button_map()
{
  std::map<std::string, std::function<ButtonValue(const joy_common_msgs::msg::Buttons &)>>
    button_getters;

  // --- Bool 型のフィールド ---
  button_getters["circle"] = [](const joy_common_msgs::msg::Buttons & msg) { return msg.circle; };
  button_getters["cross"] = [](const joy_common_msgs::msg::Buttons & msg) { return msg.cross; };
  button_getters["triangle"] = [](const joy_common_msgs::msg::Buttons & msg) {
    return msg.triangle;
  };
  button_getters["square"] = [](const joy_common_msgs::msg::Buttons & msg) { return msg.square; };
  button_getters["up"] = [](const joy_common_msgs::msg::Buttons & msg) { return msg.up; };
  button_getters["down"] = [](const joy_common_msgs::msg::Buttons & msg) { return msg.down; };
  button_getters["left"] = [](const joy_common_msgs::msg::Buttons & msg) { return msg.left; };
  button_getters["right"] = [](const joy_common_msgs::msg::Buttons & msg) { return msg.right; };
  button_getters["l_stick"] = [](const joy_common_msgs::msg::Buttons & msg) { return msg.l_stick; };
  button_getters["r_stick"] = [](const joy_common_msgs::msg::Buttons & msg) { return msg.r_stick; };
  button_getters["l1"] = [](const joy_common_msgs::msg::Buttons & msg) { return msg.l1; };
  button_getters["r1"] = [](const joy_common_msgs::msg::Buttons & msg) { return msg.r1; };
  button_getters["select"] = [](const joy_common_msgs::msg::Buttons & msg) { return msg.select; };
  button_getters["start"] = [](const joy_common_msgs::msg::Buttons & msg) { return msg.start; };
  button_getters["ps"] = [](const joy_common_msgs::msg::Buttons & msg) { return msg.ps; };

  // --- Float64 型のフィールド ---
  button_getters["l2"] = [](const joy_common_msgs::msg::Buttons & msg) { return msg.l2; };
  button_getters["r2"] = [](const joy_common_msgs::msg::Buttons & msg) { return msg.r2; };

  return button_getters;
}

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
