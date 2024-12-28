/**
 * @file imu_driver.hpp
 * @brief imu driver
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details 深度データを取得し，トピックに流す
 * ********************************************/

#ifndef _IMU_DRIVER_HPP
#define _IMU_DRIVER_HPP

#include "imu_driver/g366.hpp"

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.hpp>

#include <driver_msgs/msg/imu.hpp>

#include <string>

/**
 * @namespace imu_driver
 * @brief For imu driver
 */
namespace imu_driver
{

/**
 * @brief IMU driver class
 */
class IMUDriver : public rclcpp::Node
{
public:
  /**
   * @brief G366 & ROS-topic setting
   * @details The G366 serial setting and Define a callback function to flow the aquired data to the
   * topic at 100ms interbal
   */
  IMUDriver();

  /**
   * @brief Hardware reset of IMU
   */
  void hw_reset();

private:
  std::string portname;
  int baudrate;

  std::shared_ptr<g366::G366> g366_;
  // std::shared_ptr<g366::G366HWReseter> g366_hw_resetter_;
  rclcpp::Publisher<driver_msgs::msg::IMU>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void _setup();
  void _update();
};

}  // namespace imu_driver

#endif
