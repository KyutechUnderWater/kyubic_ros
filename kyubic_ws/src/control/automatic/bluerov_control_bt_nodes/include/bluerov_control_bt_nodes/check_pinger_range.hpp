#ifndef _CHECK_PINGER_RANGE_HPP
#define _CHECK_PINGER_RANGE_HPP

#include <behaviortree_cpp/condition_node.h>

#include <localization_msgs/msg/odometry.hpp>
#include <planner_msgs/msg/pinger_position.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief odom と pinger_position の 3D 距離が閾値以下かを判定する Condition ノード。
 */
class CheckPingerRange : public BT::ConditionNode
{
public:
  CheckPingerRange(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<planner_msgs::msg::PingerPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr odom_sub_;
  planner_msgs::msg::PingerPosition::SharedPtr latest_position_;
  localization_msgs::msg::Odometry::SharedPtr latest_odom_;

  void positionCallback(const planner_msgs::msg::PingerPosition::SharedPtr msg);
  void odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace bluerov_control_bt_nodes

#endif
