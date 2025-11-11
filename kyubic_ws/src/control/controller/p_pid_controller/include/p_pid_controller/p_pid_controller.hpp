/**
 * @file p_pid_controller.hpp
 * @brief P-PID Contorller for kyubic
 * @author R.Ohnishi
 * @date 2025/07/15
 *
 * @details x, y, z, roll, yaw軸のPID制御
 ***************************************/

#ifndef _P_PID_CONTROLLER_HPP
#define _P_PID_CONTROLLER_HPP

#include <array>
#include <map>
#include <memory>
#include <pid_controller/p_pid.hpp>

/**
 * @namespace controller
 * @brief For controller
 */
namespace controller
{

/**
 * @brief P-PID Controller class
 */
class P_PIDController
{
private:
  std::shared_ptr<pid_controller::VelocityP_PID> pid_x_, pid_y_, pid_z_, pid_roll_, pid_yaw_;

  // Define unique name(axes)
  std::array<std::string, 5> name = {"x", "y", "z", "roll", "yaw"};
  std::map<std::string, pid_controller::VelocityP_PIDParameter> pid_params;
  double roll_sf;

  /**
   * @brief Load gain from yaml file
   * @details Parse yaml and Load PID gain
   */
  bool _load_gain_from_yaml(const std::string & yaml_path);

public:
  explicit P_PIDController(const std::string & yaml_path);

  double pid_x_update(double current_slave, double current_master, double target_master);
  double pid_y_update(double current_slave, double current_master, double target_master);
  double pid_z_update(double current_slave, double current_master, double target_master);
  double pid_roll_update(double current_slave, double current_master, double target);
  double pid_yaw_update(double current_slave, double current_master, double target);

  void set_z_offset(double force);
  void set_roll_offset(double roll_deg);

  void pid_x_reset();
  void pid_y_reset();
  void pid_z_reset();
  void pid_roll_reset();
  void pid_yaw_reset();

  std::array<double, 5> update(std::array<std::array<double, 3>, 5> data);
};

}  // namespace controller

#endif
