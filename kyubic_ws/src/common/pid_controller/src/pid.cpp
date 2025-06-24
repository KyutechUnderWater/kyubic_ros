/**
 * @file pid.cpp
 * @brief PID Controller library
 * @author R.Ohnishi
 * @date 2025/05/12
 *
 * @details 位置型・速度型PIDライブラリ
 **************************************************/

#include "pid_controller/pid.hpp"

#include <algorithm>
#include <array>

namespace pid_controller
{

PositionPID::PositionPID(const double kp, const double ki, const double kd, double kf = 0.0)
: kp(kp), ki(ki), kd(kd), kf(kf)
{
}

double PositionPID::update(double current, double target, double last_saturated = 0.0)
{
  auto now = std::chrono::high_resolution_clock::now();
  double dt = std::chrono::duration_cast<std::chrono::microseconds>(now - pre_time).count() * 1e-6;
  pre_time = now;

  p = target - current;
  i += p * dt;
  d = (p - pre_p) / dt;

  // anti-windup (back-calculation, 自動整合)
  if (last_saturated != 0) {
    i -= std::min(pre_i, last_saturated);
  }

  // lowpass filter for derivation term
  d = kf * pre_d + (1 - kf) * d;

  pre_p = p;
  pre_i = i;
  pre_d = d;

  return kp * p + ki * i + kd * d;
}

std::array<double, 3> PositionPID::get_each_term()
{
  double term[3] = {p, i, d};
  return std::to_array(term);
}

void PositionPID::reset_integral() { i = 0.0; }

VelocityPID::VelocityPID(
  const double kp, const double ki, const double kd, const double kf, const double lo,
  const double hi)
: kp(kp), ki(ki), kd(kd), kf(kf), lo(lo), hi(hi)
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

  // lowpass fillter for derivation term
  d = kf * pre_d + (1 - kf) * d;

  double u = pre_u + kp * p + ki * i + kd * d;
  if (u < lo) {
    u = lo;
  }
  if (u > hi) {
    u = hi;
  }

  pre_error = error;
  pre_p = p;
  pre_d = d;
  pre_u = u;

  return u;
}

std::array<double, 3> VelocityPID::get_each_term()
{
  double term[3] = {p, i, d};
  return std::to_array(term);
}

}  // namespace pid_controller
