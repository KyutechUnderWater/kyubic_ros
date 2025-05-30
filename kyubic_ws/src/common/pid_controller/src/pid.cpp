/**
 * @file pid.cpp
 * @brief PID Controller library
 * @author R.Ohnishi
 * @date 2025/05/12
 *
 * @details 位置型・速度型PIDライブラリ
 **************************************************/

#include "pid_controller/pid.hpp"

namespace pid_controller
{

PositionPID::PositionPID(const double kp, const double ki, const double kd) : kp(kp), ki(ki), kd(kd)
{
}

double PositionPID::update(double current, double target)
{
  auto now = std::chrono::high_resolution_clock::now();
  double dt = std::chrono::duration_cast<std::chrono::microseconds>(now - pre_time).count() * 1e-6;
  pre_time = now;

  p = target - current;
  i += p * dt;
  d = (p - pre_p) / dt;

  pre_p = p;

  return kp * p + ki * i + kd * d;
}

void PositionPID::reset_integral()
{
  i = 0.0;
}

VelocityPID::VelocityPID(const double kp, const double ki, const double kd) : kp(kp), ki(ki), kd(kd)
{
}

double VelocityPID::update(double current, double target)
{
  auto now = std::chrono::high_resolution_clock::now();
  double dt = std::chrono::duration_cast<std::chrono::microseconds>(now - pre_time).count() * 1e-6;
  pre_time = now;

  double error = target - current;
  p = (error - pre_error);
  i = error * dt;
  d = (p - pre_p) / dt;

  double u = pre_u + kp * p + ki * i + kd * d;

  pre_error = error;
  pre_p = p;
  pre_u = u;

  return u;
}

}  // namespace pid_controller
