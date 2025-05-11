/**
 * @file pid.hpp
 * @brief PID Controller library
 * @author R.Ohnishi
 * @date 2025/05/12
 *
 * @details PIDライブラリ
 **************************************************/

#ifndef _PID_HPP
#define _PID_HPP

#include <chrono>

namespace pid_controller
{

class PID
{
private:
  double kp, ki, kd;
  double p, i, d;

  double pre_p = 0;
  std::chrono::high_resolution_clock::time_point pre_time =
    std::chrono::high_resolution_clock::now();

public:
  explicit PID(const double kp, const double ki, const double kd);

  void reset_i();

  double update(double current, double target);
};

}  // namespace pid_controller

#endif
