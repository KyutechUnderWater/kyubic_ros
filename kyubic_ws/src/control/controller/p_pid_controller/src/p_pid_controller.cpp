/**
 * @file p_pid_controller.cpp
 * @brief P-PID Contorller for kyubic
 * @author R.Ohnishi
 * @date 2025/07/15
 *
 * @details x, y, z, roll, yaw軸のPID制御
 ***************************************/

#include "p_pid_controller/p_pid_controller.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <iostream>
#include <memory>

namespace controller
{

P_PIDController::P_PIDController(const std::string & yaml_path)
{
  if (!_load_gain_from_yaml(yaml_path)) {
    throw std::runtime_error("Don't load yaml file with pid gain.");
  }
  pid_x_ = std::make_shared<pid_controller::VelocityP_PID>(pid_params["x"]);
  pid_y_ = std::make_shared<pid_controller::VelocityP_PID>(pid_params["y"]);
  pid_z_ = std::make_shared<pid_controller::VelocityP_PID>(pid_params["z"]);
  pid_roll_ = std::make_shared<pid_controller::VelocityP_PID>(pid_params["roll"]);
  pid_yaw_ = std::make_shared<pid_controller::VelocityP_PID>(pid_params["yaw"]);
}

bool P_PIDController::_load_gain_from_yaml(const std::string & yaml_path)
{
  std::map<std::string, pid_controller::VelocityP_PIDParameter> pid_params;

  try {
    // Load Yaml File
    std::cout << "Loading " << yaml_path << std::endl;
    YAML::Node config = YAML::LoadFile(yaml_path);

    // x, y, z, roll, yaw
    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
      std::string key = it->first.as<std::string>();
      const YAML::Node & value_node = it->second;

      auto param = pid_controller::VelocityP_PIDParameter();

      // Load master gain
      param.k = value_node["k"].as<double>();
      param.lo = value_node["lo"].as<double>();
      param.hi = value_node["hi"].as<double>();

      // Load slave gain
      const YAML::Node & vpid_node = value_node["vpid"];
      param.vpid_param.kp = vpid_node["kp"].as<double>();
      param.vpid_param.ki = vpid_node["ki"].as<double>();
      param.vpid_param.kd = vpid_node["kd"].as<double>();
      param.vpid_param.kf = vpid_node["kf"].as<double>();
      param.vpid_param.lo = vpid_node["lo"].as<double>();
      param.vpid_param.hi = vpid_node["hi"].as<double>();

      if (vpid_node["offset"]) {
        param.vpid_param.offset = vpid_node["offset"].as<double>();
      }

      // Store
      pid_params[key] = param;
    }
  } catch (const YAML::BadFile & e) {
    std::cerr << "Error: Could not open or parse YAML file. " << e.what() << std::endl;
    return false;
  } catch (const YAML::Exception & e) {
    std::cerr << "YAML parsing error: " << e.what() << std::endl;
    return false;
  }

  this->pid_params = pid_params;

  return true;
}

double P_PIDController::pid_x_update(
  double current_slave, double current_master, double target_master)
{
  return pid_x_->update(current_slave, current_master, target_master);
}

double P_PIDController::pid_y_update(
  double current_slave, double current_master, double target_master)
{
  return pid_y_->update(current_slave, current_master, target_master);
}

double P_PIDController::pid_z_update(
  double current_slave, double current_master, double target_master)
{
  return pid_z_->update(current_slave, current_master, target_master);
}

double P_PIDController::pid_roll_update(
  double current_slave, double current_master, double target_master)
{
  return pid_roll_->update(current_slave, current_master, target_master);
}

double P_PIDController::pid_yaw_update(
  double current_slave, double current_master, double target_master)
{
  return pid_yaw_->update(current_slave, current_master, target_master);
}

std::array<double, 5> P_PIDController::update(std::array<std::array<double, 3>, 5> data)
{
  std::array<std::shared_ptr<pid_controller::VelocityP_PID>, 5> list_pid_ = {
    pid_x_, pid_y_, pid_z_, pid_roll_, pid_yaw_};
  std::array<double, 5> u;

  for (size_t i = 0; i < list_pid_.size(); i++) {
    u.at(5) = list_pid_.at(i)->update(data.at(i).at(0), data.at(i).at(1), data.at(i).at(2));
  }
  return u;
}
}  // namespace controller
