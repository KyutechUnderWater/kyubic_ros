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

/**
 * @namespace pid_controller
 * @brief For PID controller
 */
namespace pid_controller
{

/**
 * @struct PositionP_PIDParameter
 * @brief Hyperparameter for PositionP_PID
 */
struct PositionP_PIDParameter
{
  double k;                         /// Master propotional gain
  double lo;                        /// Master minimum value
  double hi;                        /// Master maximum value
  PositionPIDParameter ppid_param;  /// Slave PositionPIDParameter
};

/**
 * @brief Position Form P-PID Controller class
 */
class PositionP_PID
{
private:
  double k, lo, hi;
  PositionPIDParameter ppid_param;

  std::shared_ptr<PositionPID> slave_pid_;

  double master_out = 0.0;

public:
  /**
   * @brief Controller gain setting
   * @param param Hyperparameter for position p-pid
   */
  explicit PositionP_PID(const PositionP_PIDParameter & param_);

  /**
   * @brief Update PID cycle
   * @param current_slave Slave process value
   * @param current_master Master process value
   * @param target_master Setting value
   * @param last_saturated Whether the last control input was saturated (for anti-windup)
   * @return Manipulated value
   * @details Calculate PID
   */
  double update(
    double current_slave, double current_master, double target_master, double last_saturated);

  /**
  * @brief Get calculated master output
  * @return master ouput
  */
  double get_master_out();

  /**
   * @brief Reset integral term
   * @return none
   */
  void reset();
};

/**
 * @struct PositionP_PIDParameter
 * @brief Hyperparameter for PositionP_PID
 */
struct VelocityP_PIDParameter
{
  double k;   /// Master propotional gain
  double lo;  /// Master minimum value
  double hi;  /// Master maximum value
  VelocityPIDParameter vpid_param;
};

/**
 * @brief Velocity Form P-PID Controller class
 */
class VelocityP_PID
{
private:
  double k, lo, hi;
  VelocityPIDParameter vpid_param;

  std::shared_ptr<VelocityPID> slave_pid_;

  double master_out = 0.0;

public:
  /**
   * @brief Controller gain setting
   * @param param Hyperparameter for velocity p-pid
   */
  explicit VelocityP_PID(const VelocityP_PIDParameter & param_);

  /**
   * @brief Update PID cycle
   * @param current_slave Slave process value
   * @param current_master master process value
   * @param target_master Setting value
   * @return Manipulated value
   * @details Calculate PID
   */
  double update(double current_slave, double current_master, double target_master);

  /**
  * @brief Get calculated master output
  * @return master ouput
  */
  double get_master_out();
};

}  // namespace pid_controller

#endif  // !_P_PID_HPP
