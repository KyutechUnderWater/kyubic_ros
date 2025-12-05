/**
 * @file joy2wrench.hpp
 * @brief manual control library
 * @author R.Ohnishi
 * @date 2024/11/27
 *
 * @details joy stickの情報をロボットの操作量に変換
 **************************************************/

#ifndef _JOY2WRENCH_HPP
#define _JOY2WRENCH_HPP

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "joy_common_msgs/msg/joy.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace joy2wrench
{

class Joy2WrenchStamped : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Joy2WrenchStamped(const rclcpp::NodeOptions & options);

  // Lifecycle Callback
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  double force_x_scale, force_y_scale, force_z_scale;
  double torque_x_scale, torque_z_scale;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::Subscription<joy_common_msgs::msg::Joy>::SharedPtr sub_;

  void _joyCallback(const joy_common_msgs::msg::Joy::SharedPtr msg);
};

}  // namespace joy2wrench

#endif
