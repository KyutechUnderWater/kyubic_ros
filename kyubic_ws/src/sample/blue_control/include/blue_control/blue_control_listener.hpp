#ifndef BLUE_CONTROL__BLUE_CONTROL_LISTENER_HPP_
#define BLUE_CONTROL__BLUE_CONTROL_LISTENER_HPP_

#include "blue_rov_msgs/msg/dvl75.hpp"
#include "driver_msgs/msg/depth.hpp"
#include "driver_msgs/msg/dvl.hpp"
#include "driver_msgs/msg/imu.hpp"
#include "driver_msgs/msg/power_state.hpp"
#include "driver_msgs/msg/vehicle_state.hpp"
#include "localization_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace blue_control
{

// Receive-only diagnostic node: subscribes to the topics published by the BlueROV driver stack
// (mavlink_driver, dvl75_driver) and localization to confirm they are reachable. It never
// creates a publisher or service client, so it cannot send commands to the vehicle
// (no robot_force / heartbeat / set_armed). Do not add any of those here without an explicit
// safety review — the vehicle may be physically connected and armed while this runs.
class BlueControlListener : public rclcpp::Node
{
public:
  explicit BlueControlListener(const rclcpp::NodeOptions & options);

private:
  void on_imu(const driver_msgs::msg::IMU::SharedPtr msg);
  void on_depth(const driver_msgs::msg::Depth::SharedPtr msg);
  void on_dvl(const driver_msgs::msg::DVL::SharedPtr msg);
  void on_dvl75(const blue_rov_msgs::msg::DVL75::SharedPtr msg);
  void on_vehicle_state(const driver_msgs::msg::VehicleState::SharedPtr msg);
  void on_power_state(const driver_msgs::msg::PowerState::SharedPtr msg);
  void on_odom(const localization_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<driver_msgs::msg::IMU>::SharedPtr imu_sub_;
  rclcpp::Subscription<driver_msgs::msg::Depth>::SharedPtr depth_sub_;
  rclcpp::Subscription<driver_msgs::msg::DVL>::SharedPtr dvl_sub_;
  rclcpp::Subscription<blue_rov_msgs::msg::DVL75>::SharedPtr dvl75_sub_;
  rclcpp::Subscription<driver_msgs::msg::VehicleState>::SharedPtr vehicle_state_sub_;
  rclcpp::Subscription<driver_msgs::msg::PowerState>::SharedPtr power_state_sub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace blue_control

#endif  // BLUE_CONTROL__BLUE_CONTROL_LISTENER_HPP_
