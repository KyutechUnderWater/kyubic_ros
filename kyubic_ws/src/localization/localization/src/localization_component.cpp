/**
 * @file localization_component.cpp
 * @brief localization using dvl , imu, and depth
 * @author R.Ohnishi
 * @date 2025/05/30
 *
 * @details DVL，IMU，Depthセンサを用いた自己位置推定
 ****************************************************/

#include "localization/localization_component.hpp"

#include <functional>
#include <future>
#include <geodetic_converter/geodetic_converter.hpp>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <utility>

#include "driver_msgs/msg/gnss.hpp"
#include "localization_msgs/srv/reset.hpp"

using namespace std::chrono_literals;

namespace localization
{

Localization::Localization(const rclcpp::NodeOptions & options) : Node("localization", options)
{
  coord_system_id = this->declare_parameter("coord_system_id", 1);
  geo_converter_ = std::make_shared<common::GeodeticConverter>(coord_system_id);
  origin_geodetic = geo_converter_->getOrigin();

  bool depth_enable = this->declare_parameter("depth", false);
  bool imu_enable = this->declare_parameter("imu", false);
  bool dvl_enable = this->declare_parameter("dvl", false);
  enabled_sensor |= depth_enable << 2 | imu_enable << 1 | (imu_enable ? dvl_enable : 0);

  gnss_msg_ = std::make_shared<driver_msgs::msg::Gnss>();
  imu_raw_msg_ = std::make_shared<driver_msgs::msg::IMU>();
  odom_msg_ = std::make_shared<localization_msgs::msg::Odometry>();
  odom_msg_->pose.global_pos.coordinate_system_id = coord_system_id;

  rclcpp::QoS qos(rclcpp::KeepLast(1));

  // Create publisher
  pub_odom_ = create_publisher<localization_msgs::msg::Odometry>("odom", qos);
  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/fixed_locations", 10);

  // Create subscription
  sub_depth_ = create_subscription<localization_msgs::msg::Odometry>(
    "depth/odom", qos, std::bind(&Localization::depth_callback, this, std::placeholders::_1));
  sub_imu_ = create_subscription<localization_msgs::msg::Odometry>(
    "imu/transformed", qos, std::bind(&Localization::imu_callback, this, std::placeholders::_1));
  sub_dvl_ = create_subscription<localization_msgs::msg::Odometry>(
    "dvl/odom", qos, std::bind(&Localization::dvl_callback, this, std::placeholders::_1));
  sub_gnss_ = create_subscription<driver_msgs::msg::Gnss>(
    "gnss", qos, std::bind(&Localization::gnss_callback, this, std::placeholders::_1));
  sub_imu_raw_ = create_subscription<driver_msgs::msg::IMU>(
    "imu", qos, std::bind(&Localization::imu_raw_callback, this, std::placeholders::_1));

  // Create server
  srv_ = create_service<localization_msgs::srv::Reset>(
    "reset",
    std::bind(&Localization::reset_callback, this, std::placeholders::_1, std::placeholders::_2),
    qos, nullptr);

  // Create client
  client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_depth_ = create_client<std_srvs::srv::Trigger>("depth/reset", qos, client_cb_group_);
  client_imu_ = create_client<std_srvs::srv::Trigger>("imu/reset", qos, client_cb_group_);
  client_dvl_ = create_client<std_srvs::srv::Trigger>("dvl/reset", qos, client_cb_group_);

  // Create wall timer
  timer_ = create_wall_timer(100ms, std::bind(&Localization::publisher, this));
}

void Localization::depth_callback(const localization_msgs::msg::Odometry::UniquePtr msg)
{
  odom_msg_->header = msg->header;
  odom_msg_->status.depth = msg->status.depth;
  odom_msg_->pose.position.z_depth = msg->pose.position.z_depth;
  odom_msg_->twist.linear.z_depth = msg->twist.linear.z_depth;

  all_updated |= 4;
  RCLCPP_DEBUG(this->get_logger(), "Updated Depth odometry");
}

void Localization::imu_callback(const localization_msgs::msg::Odometry::UniquePtr msg)
{
  odom_msg_->header = msg->header;
  odom_msg_->status.imu = msg->status.imu;
  odom_msg_->pose.orientation = msg->pose.orientation;
  odom_msg_->twist.angular = msg->twist.angular;

  all_updated |= 2;
  RCLCPP_DEBUG(this->get_logger(), "Updated IMU transformed");
}

void Localization::dvl_callback(const localization_msgs::msg::Odometry::UniquePtr msg)
{
  odom_msg_->header = msg->header;
  odom_msg_->status.dvl = msg->status.dvl;

  odom_msg_->pose.position.x = msg->pose.position.x;
  odom_msg_->pose.position.y = msg->pose.position.y;
  odom_msg_->pose.position.z_altitude = msg->pose.position.z_altitude;

  odom_msg_->pose.orientation = msg->pose.orientation;
  odom_msg_->twist.angular = msg->twist.angular;

  odom_msg_->twist.linear.x = msg->twist.linear.x;
  odom_msg_->twist.linear.y = msg->twist.linear.y;
  odom_msg_->twist.linear.z_altitude = msg->twist.linear.z_altitude;

  all_updated |= 1;
  RCLCPP_DEBUG(this->get_logger(), "Updated DVL odometry");
}

void Localization::gnss_callback(driver_msgs::msg::Gnss::UniquePtr msg)
{
  gnss_msg_ = std::move(msg);
  gnss_updated = true;
  RCLCPP_DEBUG(this->get_logger(), "Updated GNSS data");
}

void Localization::imu_raw_callback(driver_msgs::msg::IMU::UniquePtr msg)
{
  imu_raw_msg_ = std::move(msg);
  RCLCPP_DEBUG(this->get_logger(), "Updated IMU data");
}

void Localization::_calc_global_pose(const localization_msgs::msg::Odometry::SharedPtr odom_)
{
  double azimuth_rad = (azimuth + reference_plane.meridian_convergence) * std::numbers::pi / 180;
  double plane_x =
    odom_->pose.position.x * cos(azimuth_rad) - odom_->pose.position.y * sin(azimuth_rad);
  double plane_y =
    odom_->pose.position.x * sin(azimuth_rad) + odom_->pose.position.y * cos(azimuth_rad);

  plane_x += reference_plane.x;
  plane_y += reference_plane.y;

  common::Geodetic current_geodetic = geo_converter_->xy2geo({plane_x, plane_y, 0.0, 0.0});

  auto & global_pos_msg = odom_->pose.global_pos;
  global_pos_msg.current_pose.plane_x = plane_x;
  global_pos_msg.current_pose.plane_y = plane_y;
  global_pos_msg.current_pose.latitude = current_geodetic.latitude;
  global_pos_msg.current_pose.longitude = current_geodetic.longitude;
  global_pos_msg.ref_pose.latitude = reference_geodetic.latitude;
  global_pos_msg.ref_pose.longitude = reference_geodetic.longitude;
  global_pos_msg.ref_pose.plane_x = reference_plane.x;
  global_pos_msg.ref_pose.plane_y = reference_plane.y;
  global_pos_msg.ref_pose.meridian_convergence = reference_plane.meridian_convergence;
  global_pos_msg.azimuth = azimuth;

  std::array<common::PlaneXY, 6> xy_geo;
  for (int i = 0; i < 4; i++) {
    common::Geodetic anchor;
    anchor.latitude = geo_converter_->dms2deg(ANCHOR_POINTS_DMS[i][0]);
    anchor.longitude = geo_converter_->dms2deg(ANCHOR_POINTS_DMS[i][1]);
    xy_geo[i] = geo_converter_->geo2xy({anchor.latitude, anchor.longitude, 0.0, 0.0});
  }
  xy_geo[4] =
    geo_converter_->geo2xy({reference_geodetic.latitude, reference_geodetic.longitude, 0.0, 0.0});
  xy_geo[5] =
    geo_converter_->geo2xy({current_geodetic.latitude, current_geodetic.longitude, 0.0, 0.0});

  int id = 0;
  double scale = 1.0;
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "fixed_points";
  marker.id = id++;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.a = 0.8;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.lifetime = rclcpp::Duration(0, 0);
  marker_array.markers.push_back(marker);

  for (size_t i = 1; i < xy_geo.size(); i++) {
    xy_geo[i].x -= xy_geo[0].x;
    xy_geo[i].y -= xy_geo[0].y;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "fixed_points";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = xy_geo[i].x;
    marker.pose.position.y = xy_geo[i].y;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.a = 0.8;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = rclcpp::Duration(0, 0);
    marker_array.markers.push_back(marker);
  }
  marker_array.markers[0].color.r = 1.0;
  marker_array.markers[0].color.g = 1.0;
  marker_array.markers[0].color.b = 1.0;
  marker_array.markers[4].color.r = 1.0;
  marker_array.markers[4].color.g = 1.0;
  marker_array.markers[5].color.r = 0.0;
  marker_array.markers[5].color.b = 1.0;

  marker_pub_->publish(marker_array);
}

void Localization::publisher()
{
  if (all_updated != enabled_sensor) {
    RCLCPP_DEBUG(this->get_logger(), "Don't update. Wait data comming.");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Updated localization");

  auto odom_msg_buf_ = std::make_shared<localization_msgs::msg::Odometry>(*odom_msg_);
  _calc_global_pose(odom_msg_buf_);

  // Copy object (shared_ptr -> unique_ptr)
  auto odom_msg = std::make_unique<localization_msgs::msg::Odometry>(*odom_msg_buf_);
  pub_odom_->publish(std::move(odom_msg));

  // Reset flag
  all_updated = 0b11111000;
}

void Localization::reset_callback(
  const localization_msgs::srv::Reset::Request::SharedPtr reqest,
  const localization_msgs::srv::Reset::Response::SharedPtr response)
{
  azimuth = imu_raw_msg_->orient.z + reqest->azimuth;
  reference_geodetic.latitude = gnss_msg_->fix.latitude;
  reference_geodetic.longitude = gnss_msg_->fix.longitude;
  reference_plane =
    geo_converter_->geo2xy({reference_geodetic.latitude, reference_geodetic.longitude, 0.0, 0.0});

  int result = ~this->reset();

  // Check reset
  std::string error_msg = "";
  if (result & 1) error_msg += "DVL didn't reset\n";
  if ((result >> 1) & 1) error_msg += "IMU didn't reset\n";
  if ((result >> 2) & 1) error_msg += "Depth didn't reset\n";

  // Prepare response message
  if (error_msg == "") {
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "The reset was successful");
  } else {
    response->success = false;
    response->message = error_msg;
    RCLCPP_ERROR(this->get_logger(), "Failed to reset (%s)", error_msg.c_str());
  }
}

bool Localization::_is_server_active(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_)
{
  if (client_->wait_for_service(1s)) return true;

  //　In case of timeout
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
  } else {
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  return false;
}

FutureAndRequestId Localization::_reset_request(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_)
{
  // Send request
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  FutureAndRequestId result_future = client_->async_send_request(request).share();
  RCLCPP_INFO(this->get_logger(), "Send request to %s", client_->get_service_name());

  return result_future;
}

bool Localization::_reset_response(
  FutureAndRequestId future_, std::chrono::duration<long, std::ratio<1, 1000>> dulation,
  std::string service_name)
{
  std::future_status status = future_.wait_for(dulation);
  if (status == std::future_status::ready) {
    if (future_.get()->success) {
      RCLCPP_INFO(this->get_logger(), "The %s was successful!", service_name.c_str());
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "The %s failed!", service_name.c_str());
    }
  } else if (status == std::future_status::timeout) {
    RCLCPP_ERROR(this->get_logger(), "Timeout: Failed to call service '%s'", service_name.c_str());
  }
  return false;
}

int Localization::reset()
{
  FutureAndRequestId depth_future_, imu_future_, dvl_future_;
  if (_is_server_active(client_depth_)) depth_future_ = _reset_request(client_depth_);
  if (_is_server_active(client_imu_)) imu_future_ = _reset_request(client_imu_);
  if (_is_server_active(client_dvl_)) dvl_future_ = _reset_request(client_dvl_);

  int result = 0;
  result |= static_cast<int>(_reset_response(depth_future_, 5000ms, "depth/reset")) << 2;
  result |= static_cast<int>(_reset_response(imu_future_, 5000ms, "imu/reset")) << 1;
  result |= static_cast<int>(_reset_response(dvl_future_, 5000ms, "dvl/reset"));

  return result;
}

}  // namespace localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::Localization)
