/**
 * @file check_battery_state.hpp
 * @brief Check battery voltage state
 * @author J.Miura
 * @date 2025/11/25
 *
 * @details バッテリ電圧を取得し、4セル/6セルを自動判別して低電圧チェックを行う
 *****************************************************************************/

#ifndef _BATTERY_CHECK_HPP
#define _BATTERY_CHECK_HPP
#include <behaviortree_cpp/condition_node.h>

#include <atomic>
#include <driver_msgs/msg/power_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @brief Condition node to check the health status of battery voltage.
 */
class BatteryCheck : public BT::ConditionNode
{
public:
  /**
   * @brief Constructor for the BatteryCheck node.
   * @param name The name of the node in the behavior tree.
   * @param config The configuration of the node.
   * @param ros_node Shared pointer to the ROS 2 node used for subscriptions.
   */
  BatteryCheck(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub,
    rclcpp::Node::SharedPtr ros_node);
  /**
   * @brief Defines the input and output ports for this node.
   * @return A list of ports.
   */
  static BT::PortsList providedPorts();
  /**
   * @brief Checks the current voltage of the battery.
   * @details Identifies 4S/6S battery based on voltage threshold (16.8V).
   * @return BT::NodeStatus::SUCCESS if voltage is healthy.
   * BT::NodeStatus::FAILURE if voltage is low.
   */
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub_;
  rclcpp::Subscription<driver_msgs::msg::PowerState>::SharedPtr power_sub_;

  std::atomic<float> logic_voltage_, act_voltage_;

  bool is_battery_good(float voltage);
  void logger(bool logic_check, bool act_check);
};
#endif  // _CHECK_BATTERY_STATE_HPP
