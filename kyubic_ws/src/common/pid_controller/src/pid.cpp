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

PositionPID::PositionPID(const PositionPIDParameter param)
: kp(param.kp), ki(param.ki), kd(param.kd), kf(param.kf), offset(param.offset)
{
}

double PositionPID::update(double current, double target, double last_saturated = 0.0)
{
  auto now = std::chrono::high_resolution_clock::now();
  double dt = std::chrono::duration_cast<std::chrono::microseconds>(now - pre_time).count() * 1e-6;
  pre_time = now;

  if (first_run) {
    first_run = false;

    // The first calculation will be based solely on the P item.
    p = target - current;
    i = 0;

    pre_p = p;
    pre_i = 0;
    pre_d = 0;
    return kp * p + offset;
  }

  // Calculate Position form PID
  p = target - current;
  i += p * dt;
  d = (p - pre_p) / dt;

  // anti-windup (back-calculation, 自動整合)
  if (last_saturated != 0) {
    i -= std::min(pre_i, last_saturated);
  }

  // lowpass filter for derivation term
  d = kf * pre_d + (1 - kf) * d;

  // Store current value
  pre_p = p;
  pre_i = i;
  pre_d = d;

  return kp * p + ki * i + kd * d + offset;
}

std::array<double, 3> PositionPID::get_each_term()
{
  double term[3] = {p, i, d};
  return std::to_array(term);
}

void PositionPID::set_offset(double offset) { this->offset = offset; }

void PositionPID::reset() { first_run = true; }

VelocityPID::VelocityPID(const VelocityPIDParameter param)
: kp(param.kp),
  ki(param.ki),
  kd(param.kd),
  kf(param.kf),
  lo(param.lo),
  hi(param.hi),
  offset(param.offset)
{
}

double VelocityPID::update(double current, double target)
{
  auto now = std::chrono::high_resolution_clock::now();
  dt = std::chrono::duration_cast<std::chrono::microseconds>(now - pre_time).count() * 1e-6;
  pre_time = now;

  if (first_run) {
    first_run = false;

    double error = target - current;
    pre_error = error;
    pre_p = 0.0;
    pre_d = 0.0;

    // Initial output
    double initial_u = kp * error;
    pre_u = std::clamp(initial_u, lo, hi);

    return std::clamp(pre_u + offset, lo, hi);
  }

  // Calculate velocity form PID
  double error = target - current;
  p = (error - pre_error);
  i = error * dt;
  d = (p - pre_p) / dt;

  // lowpass fillter for derivation term
  d = kf * pre_d + (1 - kf) * d;

  double u = pre_u + kp * p + ki * i + kd * d;
  u = std::clamp(u, lo, hi);

  // Store current value
  pre_error = error;
  pre_p = p;
  pre_d = d;
  pre_u = u;

  return std::clamp(u + offset, lo, hi);
}

void VelocityPID::set_offset(double offset) { this->offset = offset; }

void VelocityPID::reset() { first_run = true; }

std::array<double, 3> VelocityPID::get_each_term()
{
  double term[3] = {p, i, d};
  return std::to_array(term);
}

double VelocityPID::get_dt() const { return dt; }

}  // namespace pid_controller
