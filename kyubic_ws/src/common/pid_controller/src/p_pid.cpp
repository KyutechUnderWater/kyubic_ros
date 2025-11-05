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

PositionP_PID::PositionP_PID(const PositionP_PIDParameter & param_)
: k(param_.k), lo(param_.lo), hi(param_.hi), ppid_param(param_.ppid_param)
{
  slave_pid_ = std::make_shared<PositionPID>(ppid_param);
}

double PositionP_PID::update(
  double current_slave, double current_master, double target_master, double last_saturated)
{
  // Calculate master P contorller
  master_out = k * (target_master - current_master);
  master_out = std::clamp(master_out, lo, hi);

  // Clamped between lo and hi value, and Calculate slave PID controller
  return slave_pid_->update(current_slave, master_out, last_saturated);
}

double PositionP_PID::get_master_out() { return master_out; }

void PositionP_PID::set_param_offset(double offset) { slave_pid_->set_offset(offset); }

void PositionP_PID::reset() { slave_pid_->reset_integral(); }

VelocityP_PID::VelocityP_PID(const VelocityP_PIDParameter & param_)
: k(param_.k), lo(param_.lo), hi(param_.hi), vpid_param(param_.vpid_param)
{
  slave_pid_ = std::make_shared<VelocityPID>(vpid_param);
}

double VelocityP_PID::update(double current_slave, double current_master, double target_master)
{
  // Calculate master P contorller
  master_out = k * (target_master - current_master);
  master_out = std::clamp(master_out, lo, hi);

  // Clamped between lo and hi value, and Calculate slave PID controller
  return slave_pid_->update(current_slave, master_out);
}

void VelocityP_PID::set_param_offset(double offset) { slave_pid_->set_offset(offset); }

double VelocityP_PID::get_master_out() { return master_out; }

void VelocityP_PID::reset() { slave_pid_->reset(); }
}  // namespace pid_controller
