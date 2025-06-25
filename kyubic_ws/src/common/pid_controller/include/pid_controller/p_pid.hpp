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

class PositionP_PID
{
private:
  double k, kp, ki, kd, kf;
  double lo, hi;

  double pre_p = 0;
  std::chrono::high_resolution_clock::time_point pre_time =
    std::chrono::high_resolution_clock::now();

  std::shared_ptr<PositionPID> master_pid_, slave_pid_;

public:
  explicit PositionP_PID(
    const double k, const double kp, const double ki, const double kd, const double kf,
    const double lo, const double hi);

  double update(
    double current_slave, double current_master, double target_master, double last_saturated);

  void reset();
};

class VelocityP_PID
{
private:
  double k, kp, ki, kd, kf;
  double mlo, mhi, slo, shi;

  double pre_p = 0;
  std::chrono::high_resolution_clock::time_point pre_time =
    std::chrono::high_resolution_clock::now();

  std::shared_ptr<VelocityPID> master_pid_, slave_pid_;

public:
  explicit VelocityP_PID(
    const double k, const double kp, const double ki, const double kd, const double kf,
    const double mlo, const double mhi, const double slo, const double shi);

  double update(double current_slave, double current_master, double target_master);

  void reset();
};

}  // namespace pid_controller

#endif  // !_P_PID_HPP
