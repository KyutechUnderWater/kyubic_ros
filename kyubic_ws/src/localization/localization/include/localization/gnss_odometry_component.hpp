#ifndef LOCALIZATION__GNSS_ODOMETRY_COMPONENT_HPP_
#define LOCALIZATION__GNSS_ODOMETRY_COMPONENT_HPP_

#include <geodetic_converter/geodetic_converter.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "driver_msgs/msg/gnss.hpp"
#include "localization_msgs/msg/odometry.hpp"

namespace localization::gnss
{

class GnssOdometry : public rclcpp::Node
{
public:
  explicit GnssOdometry(const rclcpp::NodeOptions & options);

private:
  void gnss_callback(const driver_msgs::msg::Gnss::UniquePtr msg);
  void imu_callback(const localization_msgs::msg::Odometry::UniquePtr msg);
  void depth_callback(const localization_msgs::msg::Odometry::UniquePtr msg);

  rclcpp::Publisher<localization_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Subscription<driver_msgs::msg::Gnss>::SharedPtr sub_gnss_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_imu_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_depth_;

  std::shared_ptr<common::GeodeticConverter> geo_converter_;
  common::Geodetic origin_geo_;
  common::PlaneXY origin_xy_;
  int coord_system_id_;

  // Last received IMU orientation to attach to the odom message
  localization_msgs::msg::Odometry::SharedPtr last_imu_msg_;
  localization_msgs::msg::Odometry::SharedPtr last_depth_msg_;
};

}  // namespace localization::gnss

#endif  // LOCALIZATION__GNSS_ODOMETRY_COMPONENT_HPP_
