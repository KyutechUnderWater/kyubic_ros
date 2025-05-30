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

#include <chrono>

/**
 * @namespace pid_controller
 * @brief For PID controller
 */
namespace pid_controller
{

/**
 * @brief Position Form PID Controller class
 */
class PositionPID
{
private:
  double kp, ki, kd;
  double p, i, d;

  double pre_p = 0;
  std::chrono::high_resolution_clock::time_point pre_time =
    std::chrono::high_resolution_clock::now();

public:
  /**
   * @brief controller gain setting
   * @param kp Proportional gain
   * @param ki integral gain
   * @param kd derivative gain
   */
  explicit PositionPID(const double kp, const double ki, const double kd);

  /**
   * @brief reset integral term
   * @return none
   */
  void reset_integral();

  /**
   * @brief update PID cycle
   * @param current process value
   * @param target setting value
   * @return manipulated value
   * @details calculate PID
   */
  double update(double current, double target);
};

/**
 * @brief Velocity Form PID Controller class
 */
class VelocityPID
{
private:
  double kp, ki, kd;
  double p, i, d;

  double pre_error = 0;
  double pre_p = 0;
  double pre_u = 0;
  std::chrono::high_resolution_clock::time_point pre_time =
    std::chrono::high_resolution_clock::now();

public:
  /**
   * @brief controller gain setting
   * @param kp Proportional gain
   * @param ki integral gain
   * @param kd derivative gain
   */
  explicit VelocityPID(const double kp, const double ki, const double kd);

  /**
   * @brief update PID cycle
   * @param current process value
   * @param target setting value
   * @return manipulated value
   * @details calculate PID
   */
  double update(double current, double target);
};

}  // namespace pid_controller

#endif
