/**
 * @file emergency_surfacing.hpp
 * @brief Emergency surfacing control
 * @author S.Itozono
 * @date 2025/11/18
 *
 * @details 緊急浮上のための出力をする
 ************************************/

#ifndef _EMERGENCY_SURFACING_HPP
#define _EMERGENCY_SURFACING_HPP

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace emergency
{

class EmergencySurfacing : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit EmergencySurfacing(const rclcpp::NodeOptions & options);

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
  void publish_emergency_force();

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double surfacing_force_;
};

}  // namespace emergency

#endif  // !_EMERGENCY_SURFACING_HPP
