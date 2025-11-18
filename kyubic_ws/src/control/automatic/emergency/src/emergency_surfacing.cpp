/**
 * @file emergency_surfacing.cpp
 * @brief Emergency surfacing control
 * @author S.Itozono
 * @date 2025/11/18
 *
 * @details 緊急浮上のための出力をする
 ************************************/

#include "emergency/emergency_surfacing.hpp"

using namespace std::chrono_literals;

namespace emergency
{

EmergencySurfacing::EmergencySurfacing(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("emergency_surfacing", options)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EmergencySurfacing::on_configure(const rclcpp_lifecycle::State &)
{
  surfacing_force_ = this->declare_parameter("surfacing_force", -5.0);

  rclcpp::QoS qos(rclcpp::KeepLast(10));

  pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("robot_force", qos);

  timer_ =
    this->create_wall_timer(100ms, std::bind(&EmergencySurfacing::publish_emergency_force, this));

  timer_->cancel();

  RCLCPP_DEBUG(get_logger(), "Configuration successful. Surfacing Force: %.2f", surfacing_force_);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EmergencySurfacing::on_activate(const rclcpp_lifecycle::State & state)
{
  // Activate pubblisher
  LifecycleNode::on_activate(state);

  timer_->reset();

  RCLCPP_DEBUG(get_logger(), "EMERGENCY ACTIVATED! Surfacing initiated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EmergencySurfacing::on_deactivate(const rclcpp_lifecycle::State & state)
{
  timer_->cancel();

  // Deactivate publisher
  LifecycleNode::on_deactivate(state);

  RCLCPP_DEBUG(get_logger(), "Deactivation successful. Emergency stop released.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EmergencySurfacing::on_cleanup(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  pub_.reset();
  RCLCPP_DEBUG(get_logger(), "Cleanup successful.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EmergencySurfacing::on_shutdown(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  pub_.reset();
  RCLCPP_DEBUG(get_logger(), "Shutdown successful.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void EmergencySurfacing::publish_emergency_force()
{
  // Check if LifecyclePublisher is active
  if (!pub_->is_activated()) {
    return;
  }

  auto msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();

  msg->header.stamp = this->now();
  msg->header.frame_id = "base_link";

  msg->wrench.force.x = 0.0;
  msg->wrench.force.y = 0.0;
  msg->wrench.force.z = surfacing_force_;

  msg->wrench.torque.x = 0.0;
  msg->wrench.torque.y = 0.0;
  msg->wrench.torque.z = 0.0;

  pub_->publish(std::move(msg));
}

}  // namespace emergency

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(emergency::EmergencySurfacing)
