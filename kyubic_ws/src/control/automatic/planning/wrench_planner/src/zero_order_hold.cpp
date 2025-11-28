/**
 * @file zero_order_hold.hpp
 * @brief Zero-Order-Hold for PID input term
 * @author R.Ohnishi
 * @date 2025/11/26
 *
 * @details 目標値が来ていないとき，タイムアウトするまで最後の司令を保持する
 **************************************************************************/

#include "wrench_planner/zero_order_hold.hpp"

#include <lifecycle_msgs/msg/state.hpp>

namespace planner
{

ZeroOrderHold::ZeroOrderHold(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("zero_order_hold", options)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZeroOrderHold::on_configure(const rclcpp_lifecycle::State &)
{
  timeout_ms = this->declare_parameter("timeout_ms", 1000);

  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ms * 1e6);
  plan_ = std::make_shared<planner_msgs::msg::WrenchPlan>();

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  pub_ = create_publisher<planner_msgs::msg::WrenchPlan>("goal_current_odom", qos);
  plan_sub_ = create_subscription<planner_msgs::msg::WrenchPlan>(
    "wrench_plan", qos, std::bind(&ZeroOrderHold::wrenchPlanCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<localization_msgs::msg::Odometry>(
    "odom", qos, std::bind(&ZeroOrderHold::odomCallback, this, std::placeholders::_1));

  RCLCPP_DEBUG(get_logger(), "Configuration successful.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZeroOrderHold::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activate successful");
  return LifecycleNode::on_activate(state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZeroOrderHold::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(get_logger(), "Deactivation successful. Emergency stop released.");
  return LifecycleNode::on_deactivate(state);
}

CallbackReturn ZeroOrderHold::on_cleanup(const rclcpp_lifecycle::State &)
{
  pub_.reset();
  plan_sub_.reset();
  odom_sub_.reset();
  odom_.reset();

  RCLCPP_DEBUG(get_logger(), "Cleanup successful.");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZeroOrderHold::on_shutdown(const rclcpp_lifecycle::State &)
{
  pub_.reset();
  plan_sub_.reset();
  odom_sub_.reset();

  RCLCPP_DEBUG(get_logger(), "Shutdown successful.");
  return CallbackReturn::SUCCESS;
}

void ZeroOrderHold::wrenchPlanCallback(planner_msgs::msg::WrenchPlan::SharedPtr _msg)
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  timeout_->reset(this->get_clock()->now());

  plan_ = _msg;

  if (!odom_ || !is_update) {
    RCLCPP_DEBUG(this->get_logger(), "Waiting for Odometry data... cannot publish WrenchPlan yet.");
    return;
  }

  if (copy_slave(odom_)) {
    is_update = false;
    pub_->publish(*plan_);
    RCLCPP_DEBUG(this->get_logger(), "Published WrenchPlan");
  }
}

void ZeroOrderHold::odomCallback(const localization_msgs::msg::Odometry::SharedPtr _msg)
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  is_update = true;
  odom_ = _msg;

  if (timeout_->check(this->get_clock()->now())) {
    if (copy_slave(_msg)) pub_->publish(*plan_);
  }
}

bool ZeroOrderHold::has_velocity(const planner_msgs::msg::WrenchPlan::SharedPtr _msg)
{
  auto & slave = _msg->slave;

  const double epsilon = 1e-6;
  return (std::abs(slave.x) > epsilon) && (std::abs(slave.y) > epsilon) &&
         (std::abs(slave.z) > epsilon) && (std::abs(slave.roll) > epsilon) &&
         (std::abs(slave.yaw) > epsilon);
}

bool ZeroOrderHold::copy_slave(const localization_msgs::msg::Odometry::SharedPtr _msg)
{
  plan_->slave.x = _msg->twist.linear.x;
  plan_->slave.y = _msg->twist.linear.y;
  plan_->slave.roll = _msg->twist.angular.x;
  plan_->slave.yaw = _msg->twist.angular.z;

  if (plan_->z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH) {
    plan_->slave.z = _msg->twist.linear.z_depth;
  } else if (plan_->z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_ALTITUDE) {
    plan_->slave.z = _msg->twist.linear.z_altitude;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Z mode(%d) is invalid.", plan_->z_mode);
    return false;
  }
  return true;
}

}  // namespace planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planner::ZeroOrderHold)
