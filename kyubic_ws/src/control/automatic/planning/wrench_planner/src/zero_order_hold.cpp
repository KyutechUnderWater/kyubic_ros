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
  int _hold_z_mode = this->declare_parameter("hold_z_mode", static_cast<int>(Hold_Z_Mode::DEPTH));
  timeout_ms = this->declare_parameter("timeout_ms", 1000);
  hold_z_mode = static_cast<Hold_Z_Mode>(_hold_z_mode);

  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ms * 1e6);
  hold_msg_ = std::make_shared<planner_msgs::msg::WrenchPlan>();

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  pub_ = create_publisher<planner_msgs::msg::WrenchPlan>("goal_current_odom", qos);
  plan_sub_ = create_subscription<planner_msgs::msg::WrenchPlan>(
    "zoh_wrench_plan", qos,
    std::bind(&ZeroOrderHold::wrenchPlanCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<localization_msgs::msg::Odometry>(
    "odom", qos, std::bind(&ZeroOrderHold::odomCallback, this, std::placeholders::_1));

  RCLCPP_DEBUG(get_logger(), "Configuration successful.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZeroOrderHold::on_activate(const rclcpp_lifecycle::State & state)
{
  first_timeout = true;
  had_timeout_ = false;
  timeout_->reset(this->get_clock()->now());

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

  bool had_timeout;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    timeout_->reset(this->get_clock()->now());
    had_timeout = had_timeout_;
    had_timeout_ = false;
  }

  // when return timeout, reset pid
  if (had_timeout) {
    _msg->reset = 0b00011111;
  }

  auto msg = std::make_unique<planner_msgs::msg::WrenchPlan>(*_msg);
  pub_->publish(std::move(msg));
  RCLCPP_INFO(this->get_logger(), "Published WrenchPlan");
}

void ZeroOrderHold::odomCallback(const localization_msgs::msg::Odometry::SharedPtr _msg)
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  bool is_timeout;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    is_timeout = timeout_->is_timeout(this->get_clock()->now());
    if (is_timeout) had_timeout_ = true;
  }

  if (is_timeout) {
    if (
      _msg->status.depth == localization_msgs::msg::Status::ERROR ||
      _msg->status.imu == localization_msgs::msg::Status::ERROR ||
      _msg->status.dvl == localization_msgs::msg::Status::ERROR) {
      RCLCPP_ERROR(this->get_logger(), "Odometry is invalid.");
      return;
    }

    if (first_timeout) {
      first_timeout = false;

      hold_msg_->targets.x = _msg->pose.position.x;
      hold_msg_->targets.y = _msg->pose.position.y;
      hold_msg_->targets.roll = _msg->pose.orientation.x;
      hold_msg_->targets.yaw = _msg->pose.orientation.z;
      if (hold_z_mode == Hold_Z_Mode::DEPTH) {
        hold_msg_->targets.z = _msg->pose.position.z_depth;
      } else if (hold_z_mode == Hold_Z_Mode::ALTITUDE) {
        hold_msg_->targets.z = _msg->pose.position.z_altitude;
      }

      hold_msg_->reset = 0b00011111;
    }

    if (copy_master(_msg) && copy_slave(_msg)) {
      auto msg = std::make_unique<planner_msgs::msg::WrenchPlan>(*hold_msg_);
      pub_->publish(std::move(msg));
      RCLCPP_INFO(this->get_logger(), "Published WrenchPlan (ZeroOrderHold)");
    };
  } else {
    first_timeout = true;
  }
}

bool ZeroOrderHold::copy_master(const localization_msgs::msg::Odometry::SharedPtr _msg)
{
  hold_msg_->master.x = _msg->pose.position.x;
  hold_msg_->master.y = _msg->pose.position.y;
  hold_msg_->master.roll = _msg->pose.orientation.x;
  hold_msg_->master.yaw = _msg->pose.orientation.z;

  if (hold_z_mode == Hold_Z_Mode::DEPTH) {
    hold_msg_->master.z = _msg->pose.position.z_depth;
  } else if (hold_z_mode == Hold_Z_Mode::ALTITUDE) {
    hold_msg_->master.z = _msg->pose.position.z_altitude;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Z mode(%d) is invalid.", static_cast<int>(hold_z_mode));
    return false;
  }
  return true;
}

bool ZeroOrderHold::copy_slave(const localization_msgs::msg::Odometry::SharedPtr _msg)
{
  hold_msg_->has_slave = true;
  hold_msg_->slave.x = _msg->twist.linear.x;
  hold_msg_->slave.y = _msg->twist.linear.y;
  hold_msg_->slave.roll = _msg->twist.angular.x;
  hold_msg_->slave.yaw = _msg->twist.angular.z;

  if (hold_z_mode == Hold_Z_Mode::DEPTH) {
    hold_msg_->slave.z = _msg->twist.linear.z_depth;
  } else if (hold_z_mode == Hold_Z_Mode::ALTITUDE) {
    hold_msg_->slave.z = _msg->twist.linear.z_altitude;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Z mode(%d) is invalid.", static_cast<int>(hold_z_mode));
    return false;
  }
  return true;
}

}  // namespace planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planner::ZeroOrderHold)
