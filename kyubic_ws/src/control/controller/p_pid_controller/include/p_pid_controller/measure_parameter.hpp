/**
 * @file measure_parameter.hpp
 * @brief measure offset for P_PID Controller
 * @author R.Ohnishi
 * @date 2025/11/05
 *
 * @details rollのモーメントとz-axisの浮力を計測する
 **************************************************/

#ifndef _MEASURE_PARAMETER_HPP
#define _MEASURE_PARAMETER_HPP

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <p_pid_controller/p_pid_controller.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @namespace controller
 * @brief For controller
 */
namespace controller
{

const uint buf_size = 20;

/**
 * @brief MeasureParam class
 */
class MeasureParam : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_odom_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<localization_msgs::msg::Odometry> odom_;

  std::shared_ptr<controller::P_PIDController> p_pid_ctrl_;

  const int timeout = 30;  // [s]

  std::string yaml_path;

  double target_z_depth = 0;
  double target_roll = 0;
  double force_z = 0;
  double torque_roll = 0;
  double param_z = 0;
  double param_roll = 0;

  std::array<double, buf_size> buf_sens = {0};
  std::array<double, buf_size> buf_out = {0};

  bool z_measuring = false;
  bool z_measured = false;
  bool roll_measuring = false;
  bool roll_measured = false;
  size_t count = buf_size;
  rclcpp::Time start_time;

  /**
   * @brief Save mesured parameter
   * @details write measured parameter to yaml file 
   */
  void _save_yaml();

  /**
   * @brief Acquire odometry data
   * @details Acquire odometry data via topic communication
   */
  void callback_odom(localization_msgs::msg::Odometry::UniquePtr msg);

  /**
   * @brief calculate P-PID Controller
   * @details If odometry data is updated, update pid step.
   */
  void pid_update();

  /**
   * @brief measure parameter
   * @details measure z-axis force
   * @return If true is measured, otherwise measurement is faild.
   */
  bool _measure_z();

  /**
   * @brief measure parameter
   * @details measure roll moment
   * @return If true is measured, otherwise measurement is faild.
   */
  bool _measure_roll();

  /**
   * @brief measure parameter
   * @details measure z-axis force and roll moment
   */
  void measure();

public:
  /**
   * @brief Instantiate topic and Load yaml file
   * @details Instantiat and topic(pub-sub) and Load paramete in yaml file.
   */
  explicit MeasureParam(const rclcpp::NodeOptions & options);
};

}  // namespace controller

#endif
