/**
 * @file test_p_pid.hpp
 * @brief Test code for P-PID Controller
 * @author R.Ohnishi
 * @date 2025/05/14
 *
 * @details P-PID制御のテストコード
 **************************************************/

#ifndef _TEST_PPID_HPP
#define _TEST_PPID_HPP

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <p_pid_controller/p_pid_controller.hpp>
#include <p_pid_controller_msgs/msg/targets.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @namespace controller
 * @brief For controller
 */
namespace controller
{

/**
 * @brief Test PPID class
 */
class TestPPID : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::Subscription<p_pid_controller_msgs::msg::Targets>::SharedPtr sub_targets_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_odom_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<p_pid_controller_msgs::msg::Targets> targets_;
  std::shared_ptr<localization_msgs::msg::Odometry> odom_;

  std::shared_ptr<controller::P_PIDController> p_pid_ctrl_;

  uint8_t pre_z_mode = 0;

  /// True once a real target has been received, or the first odometry sample
  /// has been used to seed a hold-current-position target.
  bool has_target_received_ = false;

  /// True once the first odometry sample has been received.
  bool has_odom_received_ = false;

  /// Wall-clock time the last odometry sample was received. Used to detect a stale
  /// odom (e.g. a flaky DVL) independently of publish_rate_hz's fixed-rate timer.
  rclcpp::Time last_odom_time_;

  /// If no odometry has been received within this many seconds, withhold
  /// robot_force instead of coasting on stale data indefinitely.
  double odom_timeout_s_ = 1.0;

  /**
   * @brief Get target value (x, y, z, roll, yaw)
   * @details Get target value via topic communication
   */
  void callback_target(const p_pid_controller_msgs::msg::Targets::UniquePtr msg);

  /**
   * @brief Acquire odometry data
   * @details Acquire odometry data via topic communication
   */
  void callback_odom(localization_msgs::msg::Odometry::UniquePtr msg);

public:
  /**
   * @brief Instantiate P-PID controller and topic
   * @details Instantiat P-PID controller and topic(pub-sub).
   */
  explicit TestPPID(const rclcpp::NodeOptions & options);

  /**
   * @brief calculate P-PID Controller
   * @details If odometry data is updated, update pid step.
   */
  void update();
};

}  // namespace controller

#endif
