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
#include <driver_msgs/msg/imu.hpp>
#include <localization_msgs/msg/odometry.hpp>

#include <array>

namespace localization
{
class DVLOdometry : public rclcpp::Node
{
private:
  rclcpp::Publisher<localization_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_imu_;
  rclcpp::Subscription<driver_msgs::msg::DVL>::SharedPtr sub_dvl_;

  rclcpp::Time pre_time;

  std::shared_ptr<localization_msgs::msg::Odometry> imu_msg_;

  std::array<double, 3> offset_angle;

  double pos_x = 0.0;
  double pos_y = 0.0;

public:
  explicit DVLOdometry(const rclcpp::NodeOptions & options);

  void _update_dvl_callback(const driver_msgs::msg::DVL::UniquePtr msg);
  void _update_imu_callback(const localization_msgs::msg::Odometry::UniquePtr msg);

  void reset(const std::array<double, 3> offset_angle = std::array<double, 3>{0.0, 0.0, 0.0});
};
}  // namespace localization

#endif
