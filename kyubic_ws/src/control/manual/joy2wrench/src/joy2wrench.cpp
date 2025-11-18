/**
 * @file joy2wrench.cpp
 * @brief manual control library
 * @author R.Ohnishi
 * @date 2024/11/27
 *
 * @details joy stickの情報をロボットの操作量に変換
 **************************************************/

#include "joy2wrench/joy2wrench.hpp"

using namespace std::chrono_literals;

namespace joy2wrench
{

Joy2WrenchStamped::Joy2WrenchStamped(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("joy_to_wrench_stamped_lifecycle", options)
{
}

/**
 * @brief "configuring" State Callback
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Joy2WrenchStamped::on_configure(const rclcpp_lifecycle::State &)
{
  force_x_scale = this->declare_parameter("force_x_scale", 1.0);
  force_y_scale = this->declare_parameter("force_y_scale", 1.0);
  force_z_scale = this->declare_parameter("force_z_scale", 1.0);
  torque_x_scale = this->declare_parameter("torque_x_scale", 1.0);
  torque_z_scale = this->declare_parameter("torque_z_scale", 1.0);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("robot_force", qos);
  sub_ = create_subscription<joy_common_msgs::msg::Joy>(
    "joy_common", 10, std::bind(&Joy2WrenchStamped::_joyCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Configuration successful. Transitioning to Inactive.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief "activating" State Callback
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Joy2WrenchStamped::on_activate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);
  RCLCPP_INFO(get_logger(), "Activation successful. Transitioning to Active.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief "deactivating" State Callback
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Joy2WrenchStamped::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivation successful. Transitioning to Inactive.");
  LifecycleNode::on_deactivate(state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief "cleaningup" State Callback
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Joy2WrenchStamped::on_cleanup(const rclcpp_lifecycle::State &)
{
  pub_.reset();
  sub_.reset();
  RCLCPP_INFO(get_logger(), "Cleanup successful. Transitioning to Unconfigured.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief "shuttingdown" State Callback
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Joy2WrenchStamped::on_shutdown(const rclcpp_lifecycle::State &)
{
  pub_.reset();
  sub_.reset();
  RCLCPP_INFO(get_logger(), "Shutdown successful.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void Joy2WrenchStamped::_joyCallback(const joy_common_msgs::msg::Joy::SharedPtr msg)
{
  if (!pub_) return;

  auto wrench_msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
  wrench_msg->header = msg->header;

  wrench_msg->wrench.force.x = msg->stick.ly * force_x_scale;
  wrench_msg->wrench.force.y = msg->stick.lx * -force_y_scale;
  wrench_msg->wrench.force.z = msg->stick.ry * -force_z_scale;

  if (msg->buttons.l2 < 1 && msg->buttons.r2) {
    wrench_msg->wrench.torque.x = -(msg->buttons.l2 - 1) * -torque_x_scale;
  } else if (msg->buttons.l2 && msg->buttons.r2 < 1) {
    wrench_msg->wrench.torque.x = -(msg->buttons.r2 - 1) * torque_x_scale;
  } else {
    wrench_msg->wrench.torque.x = 0.0;
  }

  wrench_msg->wrench.torque.y = 0.0;
  wrench_msg->wrench.torque.z = msg->stick.rx * -torque_z_scale;

  pub_->publish(std::move(wrench_msg));
}

}  // namespace joy2wrench

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(joy2wrench::Joy2WrenchStamped)
