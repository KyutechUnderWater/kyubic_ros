/**
 * @file dvl_odometry.hpp
 * @brief Calculating position by accumulating DVL velocity
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details DVLで取得した速度を累積して，位置を算出
 *********************************************************/

#ifndef _DVL_ODOMETRY_HPP
#define _DVL_ODOMETRY_HPP

#include <rclcpp/rclcpp.hpp>

#include <driver_msgs/msg/dvl.hpp>

#include <array>

namespace localization
{
class DVLOdometry : public rclcpp::Node
{
private:
  rclcpp::Subscription<driver_msgs::msg::DVL>::SharedPtr sub_;
  rclcpp::Time pre_time;

  double vel_x = 0.0;
  double vel_y = 0.0;
  double vel_z = 0.0;

  double pos_x = 0.0;
  double pos_y = 0.0;
  double pos_z = 0.0;

public:
  explicit DVLOdometry();

  void _update_callback(const driver_msgs::msg::DVL::UniquePtr msg);

  std::array<std::array<double, 3>, 3> get_odom();

  void reset();
};
}  // namespace localization

#endif
