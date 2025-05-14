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
#include <memory>

namespace pid_controller
{

P_PID::P_PID(
  const double k, const double kp, const double ki, const double kd, const double lo,
  const double hi)
: k(k), kp(kp), ki(ki), kd(kd), lo(lo), hi(hi)
{
  master_pid_ = std::make_shared<PID>(PID(k, 0.0, 0.0));
  slave_pid_ = std::make_shared<PID>(PID(kp, ki, kd));
}

double P_PID::update(double current_slave, double current_master, double target_master)
{
  double vel_ref = master_pid_->update(current_master, target_master);
  vel_ref = std::clamp(vel_ref, lo, hi);

  return slave_pid_->update(current_slave, vel_ref);
}

void P_PID::reset()
{
  master_pid_->reset_i();
  slave_pid_->reset_i();
}

}  // namespace pid_controller
