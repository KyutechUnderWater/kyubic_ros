/**
 * @file test_pid.hpp
 * @brief Test code for PID Controller
 * @author R.Ohnishi
 * @date 2025/05/14
 *
 * @details PID制御のテストコード
 **************************************************/

#ifndef _TEST_PID_HPP
#define _TEST_PID_HPP

#include <array>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <map>
#include <memory>
#include <pid_controller/p_pid.hpp>
#include <pid_controller/pid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rt_pose_plotter_msgs/msg/targets.hpp>
#include <string>

#include "geometry_msgs/msg/wrench_stamped.hpp"

/**
 * @namespace test
 * @brief For test
 */
namespace test
{

/**
 * @brief Test PID class
 */
class TestPID : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::Subscription<rt_pose_plotter_msgs::msg::Targets>::SharedPtr sub_targets_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_joy_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_odom_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Define unique name(axes)
  std::array<std::string, 5> name = {"x", "y", "z", "roll", "yaw"};

  std::map<std::string, pid_controller::VelocityP_PIDParameter> p_pid_params;
  std::map<std::string, std::shared_ptr<pid_controller::VelocityP_PID>> vp_pids;

  std::shared_ptr<rt_pose_plotter_msgs::msg::Targets> targets_;
  std::shared_ptr<geometry_msgs::msg::WrenchStamped> joy_;
  std::shared_ptr<localization_msgs::msg::Odometry> odom_;

  double current = 0;
  bool updated = false;

  /**
   * @brief Declare parameters from param.yaml
   * @details ROS 2 parameters
   */
  void _declare_parameter();

  /**
   * @brief Get target value (x, y, z, yaw)
   * @details Get target value via topic communication
   */
  void callback_target(const rt_pose_plotter_msgs::msg::Targets::UniquePtr msg);

  /**
   * @brief Acquire joy data
   * @details Acquire joy data via topic communication
   */
  void callback_joy(geometry_msgs::msg::WrenchStamped::UniquePtr msg);

  /**
   * @brief Acquire odometry data
   * @details Acquire odometry data via topic communication
   */
  void callback_odom(localization_msgs::msg::Odometry::UniquePtr msg);

public:
  /**
   * @brief Declare pid parameters, Instantiate each pid controller and topic
   * @details Declare pid gain, and filter coeficient. Instantiate position form pid, velocity form pid, and p-pid.
   * Instantiate topic(pub-sub).
   */
  explicit TestPID(const rclcpp::NodeOptions & options);

  /**
   * @brief calculate PID Controller
   * @details If odometru data is updated, update pid step.
   */
  void update();
};

}  // namespace test

#endif  // !_TEST_PID_HPP
