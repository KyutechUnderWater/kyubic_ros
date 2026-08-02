#ifndef _WRITE_PINGER_WAYPOINT_CSV_HPP
#define _WRITE_PINGER_WAYPOINT_CSV_HPP

#include <behaviortree_cpp/action_node.h>

#include <localization_msgs/msg/odometry.hpp>
#include <planner_msgs/msg/pinger_direction.hpp>
#include <planner_msgs/msg/pinger_position.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief pinger_position / pinger_direction と odom から 1 点の waypoint CSV を書き出す。
 * @details pinger_position が有効ならその方向へ step_distance_m ずつ接近。
 * 位置が未受信または score が閾値超過のときは pinger_direction による相対ステップへフォールバック。
 */
class WritePingerWaypointCSV : public BT::StatefulActionNode
{
public:
  WritePingerWaypointCSV(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<planner_msgs::msg::PingerPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<planner_msgs::msg::PingerDirection>::SharedPtr direction_sub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr odom_sub_;
  planner_msgs::msg::PingerPosition::SharedPtr latest_position_;
  planner_msgs::msg::PingerDirection::SharedPtr latest_direction_;
  localization_msgs::msg::Odometry::SharedPtr latest_odom_;

  void positionCallback(const planner_msgs::msg::PingerPosition::SharedPtr msg);
  void directionCallback(const planner_msgs::msg::PingerDirection::SharedPtr msg);
  void odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace bluerov_control_bt_nodes

#endif
