/**
 * @file gnss_global_position_component.cpp
 * @brief calculating global position
 * @author R.Ohnishi
 * @date 2025/07/30
 *
 * @details GNSS（緯度経度）と相対位置から平面直交座標系の位置を算出
 *******************************************************************/

#include "localization/gnss_global_position_component.hpp"

#include <cmath>
#include <cstdlib>

using namespace std::chrono_literals;

namespace localization::gnss
{

GnssGlobalPosition::GnssGlobalPosition(const rclcpp::NodeOptions & options)
: Node("gnss_global_position", options)
{
  coord_system_id = this->declare_parameter("coord_system_id", 1);
  azimuth_offset = this->declare_parameter("azimuth_offset", 0.0);

  geo_converter_ = std::make_shared<common::GeodeticConverter>(coord_system_id);

  gnss_msg_ = std::make_shared<driver_msgs::msg::Gnss>();
  imu_raw_msg_ = std::make_shared<driver_msgs::msg::IMU>();

  // Create callback group
  gnss_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = gnss_cb_group_;

  // Create publisher
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  pub_ = create_publisher<localization_msgs::msg::Odometry>("global_pos", qos);

  // Create subscription
  sub_gnss_ = create_subscription<driver_msgs::msg::Gnss>(
    "gnss", qos, std::bind(&GnssGlobalPosition::gnss_callback, this, std::placeholders::_1),
    sub_opt);
  sub_imu_raw_ = create_subscription<driver_msgs::msg::IMU>(
    "imu", qos, std::bind(&GnssGlobalPosition::imu_raw_callback, this, std::placeholders::_1),
    sub_opt);

  // Create service
  srv_ = create_service<std_srvs::srv::Trigger>(
    "reset",
    std::bind(
      &GnssGlobalPosition::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void GnssGlobalPosition::update_callback(const localization_msgs::msg::Odometry::UniquePtr odom_)
{
  if (odom_->status.dvl.id >= common_msgs::msg::Status::ERROR || !reset_flag) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Don't calculate global position. Because dvl odometry error or not reset");
    odom_->status.gnss.id = common_msgs::msg::Status::ERROR;
    return;
  } else
    odom_->status.gnss.id = common_msgs::msg::Status::NORMAL;

  double azimuth_rad =
    -(azimuth + ref_plane.meridian_convergence) * std::numbers::pi / 180;  // counter clock-wise(+)
  double plane_x =
    odom_->pose.position.x * cos(azimuth_rad) - odom_->pose.position.y * sin(azimuth_rad);
  double plane_y =
    odom_->pose.position.x * sin(azimuth_rad) + odom_->pose.position.y * cos(azimuth_rad);

  plane_x += ref_plane.x;
  plane_y += ref_plane.y;
  common::Geodetic current_geodetic = geo_converter_->xy2geo({plane_x, plane_y, 0.0, 0.0});

  {
    auto msg = std::make_unique<localization_msgs::msg::Odometry>(*odom_);
    msg->pose.global_pos.coordinate_system_id = coord_system_id;
    msg->pose.global_pos.azimuth = azimuth;

    msg->pose.global_pos.current_pos.latitude = current_geodetic.latitude;
    msg->pose.global_pos.current_pos.longitude = current_geodetic.longitude;
    msg->pose.global_pos.current_pos.plane_x = plane_x;
    msg->pose.global_pos.current_pos.plane_y = plane_y;
    msg->pose.global_pos.current_pos.meridian_convergence = current_geodetic.meridian_convergence;
    msg->pose.global_pos.current_pos.scale_coefficient = current_geodetic.scale_coefficient;

    msg->pose.global_pos.ref_pos.latitude = ref_geodetic.latitude;
    msg->pose.global_pos.ref_pos.longitude = ref_geodetic.longitude;
    msg->pose.global_pos.ref_pos.plane_x = ref_plane.x;
    msg->pose.global_pos.ref_pos.plane_y = ref_plane.y;
    msg->pose.global_pos.ref_pos.meridian_convergence = ref_plane.meridian_convergence;
    msg->pose.global_pos.ref_pos.scale_coefficient = ref_plane.scale_coefficient;

    pub_->publish(std::move(msg));
  }
}

void GnssGlobalPosition::gnss_callback(driver_msgs::msg::Gnss::UniquePtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mtx_gnss_);
    gnss_msg_ = std::move(msg);
  }
  gnss_updated = true;
  RCLCPP_DEBUG(this->get_logger(), "Updated GNSS data");
}

void GnssGlobalPosition::imu_raw_callback(driver_msgs::msg::IMU::UniquePtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mtx_imu_);
    imu_raw_msg_ = std::move(msg);
  }
  imu_updated = true;
  RCLCPP_DEBUG(this->get_logger(), "Updated IMU data");
}

bool GnssGlobalPosition::_waiting_update(std::atomic<bool> & updated, uint8_t timeout_count)
{
  for (int i = 0; i < timeout_count; i++) {
    if (updated) return true;
    std::this_thread::sleep_for(100ms);
  }

  return false;
}

void GnssGlobalPosition::reset_callback(
  [[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr request,
  const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  RCLCPP_INFO(this->get_logger(), "Reset");

  imu_updated = false;
  gnss_updated = false;
  reset_flag = false;

  // Updated IMU data
  if (_waiting_update(imu_updated, 2)) {
    std::lock_guard<std::mutex> lock(mtx_imu_);
    azimuth = imu_raw_msg_->orient.z + azimuth_offset;
  } else {
    response->success = false;
    response->message = "Timeout: No IMU data in 200ms.";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  // Updated GNSS data
  if (_waiting_update(gnss_updated, 30)) {
    std::lock_guard<std::mutex> lock(mtx_gnss_);
    ref_geodetic.latitude = gnss_msg_->fix.latitude;
    ref_geodetic.longitude = gnss_msg_->fix.longitude;
  } else {
    response->success = false;
    response->message = "Timeout: No GNSS data in 3s.";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  ref_plane = geo_converter_->geo2xy({ref_geodetic.latitude, ref_geodetic.longitude, 0.0, 0.0});

  reset_flag = true;
  response->success = true;
  response->message = "GNSS global position reset successfully";
}

}  // namespace localization::gnss

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::gnss::GnssGlobalPosition)
