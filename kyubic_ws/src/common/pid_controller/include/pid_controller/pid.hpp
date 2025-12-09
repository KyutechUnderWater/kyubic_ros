/**
 * @file pid.hpp
 * @brief PID Controller library
 * @author R.Ohnishi
 * @date 2025/05/12
 *
 * @details 位置型・速度型PIDライブラリ
 **************************************************/

#ifndef _PID_HPP
#define _PID_HPP

#include <array>
#include <chrono>

/**
 * @namespace pid_controller
 * @brief For PID controller
 */
namespace pid_controller
{

/**
 * @struct PositionPIDParameter
 * @brief Hyperparameter for PositionPID
 */
struct PositionPIDParameter
{
  double kp;            /// Proportional gain
  double ki;            /// Integral gain
  double kd;            /// Derivative gain
  double kf;            /// Low-pass filter coefficient
  double offset = 0.0;  /// offset of output value
};

/**
 * @brief Position Form PID Controller class
 */
class PositionPID
{
private:
  double kp, ki, kd, kf;
  double offset;
  double p, i, d;

  double pre_p = 0;
  double pre_i = 0;
  double pre_d = 0;
  std::chrono::high_resolution_clock::time_point pre_time =
    std::chrono::high_resolution_clock::now();

public:
  /**
   * @brief Controller gain setting
   * @param param Hyperparameter for velocity pid
   */
  explicit PositionPID(const PositionPIDParameter param);

  /**
   * @brief Reset integral term
   * @return none
   */
  void reset_integral();

  /**
   * @brief Get each term value of pid
   * @return array(x, y, z)
   */
  std::array<double, 3> get_each_term();

  /**
   * @brief Set offset
   * @param offset offset parameter
   */
  void set_offset(double offset);

  /**
   * @brief Update PID cycle
   * @param current Process value
   * @param target Setting value
   * @param last_saturated Whether the last control input was saturated (for anti-windup)
   * @return Manipulated value
   * @details Calculate PID
   */
  double update(double current, double target, double last_saturated);
};

/**
 * @struct VelocityPIDParameter
 * @brief Hyperparameter for VelocityPID
 */
struct VelocityPIDParameter
{
  double kp;            /// Proportional gain
  double ki;            /// Integral gain
  double kd;            /// Derivative gain
  double kf;            /// Low-pass filter coefficient
  double lo;            /// Minimal value
  double hi;            /// Maximun value
  double offset = 0.0;  /// offset of output value
};

/**
 * @brief Velocity Form PID Controller class
 */
class VelocityPID
{
private:
  double kp, ki, kd, kf;
  double lo, hi;
  double offset;
  double p, i, d;

  double pre_error = 0;
  double pre_p = 0;
  double pre_d = 0;
  double pre_u = 0;
  std::chrono::high_resolution_clock::time_point pre_time =
    std::chrono::high_resolution_clock::now();

public:
  /**
   * @brief Controller gain setting
   * @param param Hyperparameter for velocity pid
   */
  explicit VelocityPID(const VelocityPIDParameter param);

  /**
   * @brief Get each term value of pid
   * @return array(x, y, z)
   */
  std::array<double, 3> get_each_term();

  /**
   * @brief Update PID cycle
   * @param current Process value
   * @param target Setting value
   * @return Manipulated value
   * @details Calculate PID
   */
  double update(double current, double target);

  /**
   * @brief Set offset
   * @param offset offset parameter
   */
  void set_offset(double offset);

  /**
   * @brief Reset previous output
   * @return none
   */
  void reset();
};

}  // namespace pid_controller

#endif
