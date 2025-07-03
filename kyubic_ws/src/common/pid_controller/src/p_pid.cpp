/**
 * @file p_pid.cpp
 * @brief P-PID Controller library
 * @author R.Ohnishi
 * @date 2025/05/14
 *
 * @details P-PID (カスケード制御) ライブラリ
 **************************************************/

#include "pid_controller/p_pid.hpp"

namespace pid_controller
{

PositionP_PID::PositionP_PID(const std::shared_ptr<PositionP_PIDParameter> param_)
: k(param_->k), lo(param_->lo), hi(param_->hi), ppid_param(param_->ppid_param)
{
  slave_pid_ = std::make_shared<PositionPID>(ppid_param);
}

double PositionP_PID::update(
  double current_slave, double current_master, double target_master, double last_saturated)
{
  // Calculate master P contorller
  double vel_ref = k * (target_master - current_master);

  // Clamped between lo and hi value, and Calculate slave PID controller
  return slave_pid_->update(current_slave, std::clamp(vel_ref, lo, hi), last_saturated);
}

void PositionP_PID::reset() { slave_pid_->reset_integral(); }

VelocityP_PID::VelocityP_PID(const std::shared_ptr<VelocityP_PIDParameter> param_)
: k(param_->k), lo(param_->lo), hi(param_->hi), vpid_param(param_->vpid_param)
{
  slave_pid_ = std::make_shared<VelocityPID>(vpid_param);
}

double VelocityP_PID::update(double current_slave, double current_master, double target_master)
{
  // Calculate master P contorller
  double vel_ref = k * (target_master - current_master);

  // Clamped between lo and hi value, and Calculate slave PID controller
  return slave_pid_->update(current_slave, std::clamp(vel_ref, lo, hi));
}

}  // namespace pid_controller
