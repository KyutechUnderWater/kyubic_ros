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
#include <p_pid_controller/p_pid_controller.hpp>
#include <p_pid_controller_msgs/msg/targets.hpp>
#include <planner_msgs/msg/wrench_plan.hpp>
#include <rclcpp/rclcpp.hpp>

namespace planner
{

class WrenchPlanner : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::Publisher<p_pid_controller_msgs::msg::Targets>::SharedPtr pub_target_;
  rclcpp::Subscription<planner_msgs::msg::WrenchPlan>::SharedPtr sub_;

  std::shared_ptr<planner_msgs::msg::WrenchPlan> goal_current_odom_;

  std::shared_ptr<controller::P_PIDController> p_pid_ctrl_;

  uint8_t pre_z_mode = 0;

  void _update_wrench();

  void goalCurrentOdomCallback(const planner_msgs::msg::WrenchPlan::SharedPtr msg);

public:
  explicit WrenchPlanner(const rclcpp::NodeOptions & options);
};

}  // namespace planner

#endif
