#include "localization/gnss_odometry_component.hpp"

#include <cmath>
#include <functional>

namespace localization::gnss
{

GnssOdometry::GnssOdometry(const rclcpp::NodeOptions & options) : Node("gnss_odometry", options)
{
  coord_system_id_ = this->declare_parameter("coord_system_id", 1);
  double origin_lat = this->declare_parameter("origin_lat", 33.9);
  double origin_lon = this->declare_parameter("origin_lon", 130.9);

  geo_converter_ = std::make_shared<common::GeodeticConverter>(coord_system_id_);

  origin_geo_.latitude = origin_lat;
  origin_geo_.longitude = origin_lon;
  origin_xy_ = geo_converter_->geo2xy(origin_geo_);

  rclcpp::QoS qos(rclcpp::KeepLast(1));

  pub_odom_ = create_publisher<localization_msgs::msg::Odometry>("odom", qos);

  sub_gnss_ = create_subscription<driver_msgs::msg::Gnss>(
    "gnss", qos, std::bind(&GnssOdometry::gnss_callback, this, std::placeholders::_1));

  sub_imu_ = create_subscription<localization_msgs::msg::Odometry>(
    "transformed_imu", qos, std::bind(&GnssOdometry::imu_callback, this, std::placeholders::_1));

  sub_depth_ = create_subscription<localization_msgs::msg::Odometry>(
    "depth_odom", qos, std::bind(&GnssOdometry::depth_callback, this, std::placeholders::_1));
}

void GnssOdometry::imu_callback(const localization_msgs::msg::Odometry::UniquePtr msg)
{
  if (!last_imu_msg_) {
    last_imu_msg_ = std::make_shared<localization_msgs::msg::Odometry>();
  }
  *last_imu_msg_ = *msg;
}

void GnssOdometry::depth_callback(const localization_msgs::msg::Odometry::UniquePtr msg)
{
  if (!last_depth_msg_) {
    last_depth_msg_ = std::make_shared<localization_msgs::msg::Odometry>();
  }
  *last_depth_msg_ = *msg;
}

void GnssOdometry::gnss_callback(const driver_msgs::msg::Gnss::UniquePtr msg)
{
  auto odom_msg = std::make_unique<localization_msgs::msg::Odometry>();
  odom_msg->header = msg->header;

  if (msg->status.id == common_msgs::msg::Status::ERROR) {
    odom_msg->status.gnss.id = common_msgs::msg::Status::ERROR;
    pub_odom_->publish(std::move(odom_msg));
    return;
  }

  common::Geodetic current_geo;
  current_geo.latitude = msg->fix.latitude;
  current_geo.longitude = msg->fix.longitude;

  common::PlaneXY current_xy = geo_converter_->geo2xy(current_geo);

  // X is North, Y is East
  odom_msg->pose.position.x = current_xy.x - origin_xy_.x;
  odom_msg->pose.position.y = current_xy.y - origin_xy_.y;

  // Set the reference origin so the planner can use it dynamically
  odom_msg->pose.global_pos.coordinate_system_id = coord_system_id_;
  odom_msg->pose.global_pos.ref_pose.latitude = origin_geo_.latitude;
  odom_msg->pose.global_pos.ref_pose.longitude = origin_geo_.longitude;

  if (last_imu_msg_) {
    odom_msg->pose.orientation = last_imu_msg_->pose.orientation;
    odom_msg->twist.angular = last_imu_msg_->twist.angular;
    odom_msg->status.imu = last_imu_msg_->status.imu;
  }

  if (last_depth_msg_) {
    odom_msg->pose.position.z_depth = last_depth_msg_->pose.position.z_depth;
    odom_msg->twist.linear.z_depth = last_depth_msg_->twist.linear.z_depth;
    odom_msg->status.depth = last_depth_msg_->status.depth;
  }

  // Calculate speed if necessary, but GNSS alone is discrete.
  // For now, twist linear is 0 since GNSS doesn't provide direct velocity usually.
  // A particle filter would provide the smooth twist.

  pub_odom_->publish(std::move(odom_msg));
}

}  // namespace localization::gnss

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::gnss::GnssOdometry)
