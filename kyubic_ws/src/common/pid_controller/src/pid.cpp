/**
 * @file pid.cpp
 * @brief PID Controller library
 * @author R.Ohnishi
 * @date 2025/05/12
 *
 * @details PIDライブラリ
 **************************************************/

#include "pid_controller/pid.hpp"

namespace pid_controller
{

PID::PID(const double kp, const double ki, const double kd) : kp(kp), ki(ki), kd(kd)
{
}

double PID::update(double current, double target)
{
  auto now = std::chrono::high_resolution_clock::now();
  double dt = std::chrono::duration_cast<std::chrono::microseconds>(now - pre_time).count() / 1e6;
  pre_time = now;

  p = target - current;
  i += p * dt;
  d = p - pre_p;

  pre_p = p;

  return p * kp + i * ki + d * kd;
}

void PID::reset_i()
{
  i = 0.0;
}

}  // namespace pid_controller
