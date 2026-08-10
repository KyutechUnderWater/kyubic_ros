/**
 * @file wrench_planner.hpp
 * @brief plan force and torque from path
 * @author R.Ohnishi
 * @date 2025/07/15
 *
 * @details pathからforceとtorqeを計画
 *************************************/

#ifndef _WRENCH_PLANNER_HPP
#define _WRENCH_PLANNER_HPP

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <p_pid_controller/p_pid_controller.hpp>
#include <p_pid_controller_msgs/msg/targets.hpp>
#include <planner_msgs/msg/wrench_plan.hpp>
#include <rclcpp/rclcpp.hpp>

namespace planner::wrench_planner
{

class WrenchPlanner : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::Publisher<p_pid_controller_msgs::msg::Targets>::SharedPtr pub_target_;
  rclcpp::Subscription<planner_msgs::msg::WrenchPlan>::SharedPtr sub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;

  planner_msgs::msg::WrenchPlan::SharedPtr goal_current_odom_;
  localization_msgs::msg::Odometry::SharedPtr current_odom_;

  std::shared_ptr<controller::P_PIDController> p_pid_ctrl_;

  uint8_t pre_z_mode_ = 0;

  /// True once the first WrenchPlan (goal) message has been received.
  bool has_goal_received_ = false;

  /// True once the first odometry sample has been received.
  bool has_odom_received_ = false;

  /// Wall-clock time the last odometry sample was received. Used to detect a stale
  /// odom (e.g. a flaky DVL) independently of publish_rate_hz's fixed-rate timer.
  rclcpp::Time last_odom_time_;

  /// If no odometry has been received within this many seconds, withhold
  /// robot_force instead of coasting on stale data indefinitely.
  double odom_timeout_s_ = 1.0;

  void _update_wrench();

  void goalCurrentOdomCallback(const planner_msgs::msg::WrenchPlan::SharedPtr msg);
  void odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg);
  bool has_velocity(const planner_msgs::msg::WrenchPlan::SharedPtr msg);

public:
  explicit WrenchPlanner(const rclcpp::NodeOptions & options);
};

}  // namespace planner::wrench_planner

#endif
