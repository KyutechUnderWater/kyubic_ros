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

#include <pid_controller/p_pid.hpp>
#include <pid_controller/pid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "localization_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>

#include <memory>

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
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_odom_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<pid_controller::PositionPID> ppid_;
  std::shared_ptr<pid_controller::VelocityPID> vpid_;
  std::shared_ptr<pid_controller::VelocityP_PID> p_pid_;

  std::shared_ptr<localization_msgs::msg::Odometry> odom_;

  double k, kp, ki, kd, kf, mlo, mhi, slo, shi;
  double target = 0;
  double current = 0;
  bool updated = false;

  /**
   * @brief Get target value
   * @details Get target value via topic communication
   */
  void callback_target(const std_msgs::msg::Float32::UniquePtr msg);

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
