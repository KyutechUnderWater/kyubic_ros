/**
 * @file velocity_wrench_planner.hpp
 * @brief plan force and torque from path with velocity
 * @author R.Ohnishi
 * @date 2025/11/24
 *
 * @details pathからforceとtorqeを計画（速度制御）
 ************************************************/

#ifndef _VELOCITY_WRENCH_PLANNER_HPP
#define _VELOCITY_WRENCH_PLANNER_HPP

#include <p_pid_controller_msgs/msg/targets.h>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <mutex>
#include <p_pid_controller/p_pid_controller.hpp>
#include <planner_msgs/msg/wrench_plan.hpp>
#include <rclcpp/rclcpp.hpp>

namespace planner::wrench_planner
{

class VelocityWrenchPlanner : public rclcpp::Node
{
private:
  rclcpp::Publisher<planner_msgs::msg::WrenchPlan>::SharedPtr pub_;
  rclcpp::Subscription<planner_msgs::msg::WrenchPlan>::SharedPtr plan_sub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr odom_sub_;

  localization_msgs::msg::Odometry::SharedPtr odom_;
  bool is_update = false;

  std::mutex mutex_;

  void wrenchPlanCallback(planner_msgs::msg::WrenchPlan::SharedPtr msg);
  void odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg);

public:
  explicit VelocityWrenchPlanner(const rclcpp::NodeOptions & options);
};

}  // namespace planner::wrench_planner

#endif
