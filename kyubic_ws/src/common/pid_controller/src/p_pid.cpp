/**
 * @file p_pid.cpp
 * @brief P-PID Controller library
 * @author R.Ohnishi
 * @date 2025/05/14
 *
 * @details P-PID (カスケード制御) ライブラリ
 **************************************************/

#include "pid_controller/p_pid.hpp"

#include <algorithm>
#include <iostream>
#include <memory>

namespace pid_controller
{

PositionP_PID::PositionP_PID(
  const double k, const double kp, const double ki, const double kd, const double kf,
  const double lo, const double hi)
: k(k), kp(kp), ki(ki), kd(kd), kf(kf), lo(lo), hi(hi)
{
  master_pid_ = std::make_shared<PositionPID>(k, 0.0, 0.0, 0.0);
  slave_pid_ = std::make_shared<PositionPID>(kp, ki, kd, kf);
}

double PositionP_PID::update(
  double current_slave, double current_master, double target_master, double last_saturated)
{
  double vel_ref = master_pid_->update(current_master, target_master, 0.0);
  vel_ref = std::clamp(vel_ref, lo, hi);

  return slave_pid_->update(current_slave, vel_ref, last_saturated);
}

void PositionP_PID::reset()
{
  master_pid_->reset_integral();
  slave_pid_->reset_integral();
}

VelocityP_PID::VelocityP_PID(
  const double k, const double kp, const double ki, const double kd, const double kf,
  const double mlo, const double mhi, const double slo, const double shi)
: k(k), kp(kp), ki(ki), kd(kd), kf(kf), mlo(mlo), mhi(mhi), slo(slo), shi(shi)
{
  master_pid_ = std::make_shared<VelocityPID>(k, 0.0, 0.0, 0.0, mlo, mhi);
  slave_pid_ = std::make_shared<VelocityPID>(kp, ki, kd, kf, slo, shi);
}

double VelocityP_PID::update(double current_slave, double current_master, double target_master)
{
  double vel_ref = (target_master - current_master) * k;
  vel_ref = std::clamp(vel_ref, mlo, mhi);
  return slave_pid_->update(current_slave, vel_ref);
}

}  // namespace pid_controller
