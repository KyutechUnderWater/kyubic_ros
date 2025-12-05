/**
 * @file update_mode.hpp
 * @brief Get mode from joy-controller
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details Joy-Controllerからボタンの情報を取得しmodeを更新する
 **************************************************************/

#ifndef _UPDATE_MODE_HPP
#define _UPDATE_MODE_HPP

#include <behaviortree_cpp/action_node.h>

#include <cstdint>
#include <joy_common/joy_common.hpp>
#include <joy_common_msgs/msg/joy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <timer/timeout.hpp>

namespace behavior_tree
{

/**
 * @brief Action node to update the robot's operation mode based on Joystick input.
 * @details This node subscribes to a joystick topic and checks specific buttons defined
 *          in the input ports. It switches the mode between "manual" and "auto".
 */
class UpdateMode : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor for the UpdateMode node.
   * @param name The name of the node in the behavior tree.
   * @param config The configuration of the node.
   * @param ros_node Shared pointer to the ROS 2 node used for subscription.
   * @param timeout_ms joy topic timeout [ms]
   */
  UpdateMode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub,
    rclcpp::Node::SharedPtr ros_node, uint64_t timeout_ms);

  /**
   * @brief Defines the input and output ports for this node.
   * @return A list containing:
   *         - InputPort "joy_topic": Topic name for the joystick. default is "/joy_common/joy_common".
   *         - InputPort "manual_button": Name of the button to switch to manual mode.
   *         - InputPort "auto_button": Name of the button to switch to auto mode.
   *         - OutputPort "mode": The current mode ("manual" or "auto").
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Checks joystick state and updates the mode.
   * @return BT::NodeStatus::SUCCESS if executed correctly.
   *         BT::NodeStatus::FAILURE if Joy message is missing, ports are invalid, or buttons map is invalid.
   */
  BT::NodeStatus tick() override;

private:
  /**
   * @brief Helper function to check if a button/axis is pressed/active.
   * @param value The variant value (bool for buttons, double for axes) from the button map.
   * @return true if the button is pressed or axis is past the threshold.
   */
  bool _check_value(std::variant<bool, double> value);

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub_;
  rclcpp::Subscription<joy_common_msgs::msg::Joy>::SharedPtr joy_sub_;
  joy_common_msgs::msg::Joy::SharedPtr joy_msg_;
  joy_common::ButtonMap button_map_;

  joy_common::ButtonMap::iterator manual_it, auto_it;

  std::mutex mutex_;
  std::shared_ptr<timer::Timeout> timeout_;
  uint64_t timeout_ms_;
};

}  // namespace behavior_tree

#endif  // _UPDATE_MODE_HPP
