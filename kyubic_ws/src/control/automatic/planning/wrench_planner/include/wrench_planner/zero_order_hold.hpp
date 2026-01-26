/**
 * @file zero_order_hold.hpp
 * @brief Zero-Order-Hold for PID input term
 * @author R.Ohnishi
 * @date 2025/11/26
 *
 * @details 目標値が来ていないとき，タイムアウトするまで最後の司令を保持する
 **************************************************************************/

#ifndef _ZERO_ORDER_HOLD_HPP
#define _ZERO_ORDER_HOLD_HPP

#include <cstdint>
#include <localization_msgs/msg/odometry.hpp>
#include <memory>
#include <mutex>
#include <p_pid_controller_msgs/msg/base_axes.hpp>
#include <planner_msgs/msg/wrench_plan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <timer/timeout.hpp>

namespace planner::wrench_planner
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

enum class Hold_Z_Mode
{
  DEPTH = 0,
  ALTITUDE = 1,
};

class ZeroOrderHold : public rclcpp_lifecycle::LifecycleNode
{
private:
  Hold_Z_Mode hold_z_mode;
  uint64_t timeout_ms;

  rclcpp_lifecycle::LifecyclePublisher<planner_msgs::msg::WrenchPlan>::SharedPtr pub_;
  rclcpp::Subscription<planner_msgs::msg::WrenchPlan>::SharedPtr plan_sub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::shared_ptr<timer::Timeout> timeout_;

  localization_msgs::msg::Odometry::SharedPtr odom_;
  planner_msgs::msg::WrenchPlan::SharedPtr hold_msg_;
  bool first_timeout = true;
  bool had_timeout_ = true;

  std::mutex mutex_;

  void wrenchPlanCallback(planner_msgs::msg::WrenchPlan::SharedPtr _msg);
  void odomCallback(const localization_msgs::msg::Odometry::SharedPtr _msg);
  bool copy_master(const localization_msgs::msg::Odometry::SharedPtr _msg);
  bool copy_slave(const localization_msgs::msg::Odometry::SharedPtr _msg);

public:
  explicit ZeroOrderHold(const rclcpp::NodeOptions & options);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state) override;
};

}  // namespace planner::wrench_planner

#endif  // _ZERO_ORDER_HOLD_HPP
