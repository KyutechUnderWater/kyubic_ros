/**
 * @file p_pid.hpp
 * @brief P-PID Controller library
 * @author R.Ohnishi
 * @date 2025/05/14
 *
 * @details P-PID (カスケード制御) ライブラリ
 **************************************************/

#ifndef _P_PID_HPP
#define _P_PID_HPP

#include "pid.hpp"

#include <memory>

namespace pid_controller
{

struct PositionP_PIDParameter
{
  double k;
  double lo;
  double hi;
  PositionPIDParameter ppid_param;
};

class PositionP_PID
{
private:
  double k, lo, hi;
  PositionPIDParameter ppid_param;

  double pre_p = 0;
  std::chrono::high_resolution_clock::time_point pre_time =
    std::chrono::high_resolution_clock::now();

  std::shared_ptr<PositionPID> slave_pid_;

public:
  explicit PositionP_PID(const std::shared_ptr<PositionP_PIDParameter> param_);

  double update(
    double current_slave, double current_master, double target_master, double last_saturated);

  void reset();
};

struct VelocityP_PIDParameter
{
  double k;
  double lo;
  double hi;
  VelocityPIDParameter vpid_param;
};

class VelocityP_PID
{
private:
  double k, lo, hi;
  VelocityPIDParameter vpid_param;

  double pre_p = 0;
  std::chrono::high_resolution_clock::time_point pre_time =
    std::chrono::high_resolution_clock::now();

  std::shared_ptr<VelocityPID> slave_pid_;

public:
  explicit VelocityP_PID(const std::shared_ptr<VelocityP_PIDParameter> param_);

  double update(double current_slave, double current_master, double target_master);

  void reset();
};

}  // namespace pid_controller

#endif  // !_P_PID_HPP
