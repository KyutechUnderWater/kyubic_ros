/**
 * @file localization.hpp
 * @brief localization using dvl and depth
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details DVL, IMU, Depthセンサを用いた自己位置推定
 ****************************************************/

#ifndef _LOCALIZATIN_COMPONENT_HPP
#define _LOCALIZATIN_COMPONENT_HPP

#include <geodetic_converter/geodetic_converter.hpp>
#include <rclcpp/rclcpp.hpp>

#include "driver_msgs/msg/gnss.hpp"
#include "localization_msgs/msg/global_pose.hpp"
#include <driver_msgs/msg/gnss.hpp>
#include <localization_msgs/msg/global_pose.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

/**
 * @namespace localization
 * @brief localization
 */
namespace localization
{

/** Create alias for namespace */
using FutureAndRequestId = rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture;

/**
 * @brief Localization class
 */
class Localization : public rclcpp::Node
{
private:
  uint8_t coord_system_id;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::Publisher<localization_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<localization_msgs::msg::GlobalPose>::SharedPtr pub_global_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_depth_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_imu_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_dvl_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_gnss_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_depth_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_imu_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_dvl_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<localization_msgs::msg::Odometry> odom_msg_;
  std::shared_ptr<driver_msgs::msg::Gnss> gnss_msg_;
  std::shared_ptr<localization_msgs::msg::GlobalPose> global_pose_msg_;
  GSI::LatLon origin_geodetic;
  GSI::LatLon reference_geodetic;
  GSI::XY reference_plane;
  double azimuth;

  bool gnss_updated = false;
  uint8_t enabled_sensor = 0b11111000;
  uint8_t all_updated = 0b11111000;

  /**
   * @brief Update depth odometry
   * @details Acquisition the depth odometry.
   */
  void depth_callback(const localization_msgs::msg::Odometry::UniquePtr msg);

  /**
   * @brief Update imu transformed
   * @details Acquisition the imu transformed data.
   */
  void imu_callback(const localization_msgs::msg::Odometry::UniquePtr msg);

  /**
   * @brief Update dvl odometry
   * @details Acquisitionn the dvl odometry.
   */
  void dvl_callback(const localization_msgs::msg::Odometry::UniquePtr msg);

  /**
   * @brief Update gnss data
   * @details Acquisitionn the gnss data.
   */
  void gnss_callback(const driver_msgs::msg::Gnss::UniquePtr msg);

  /**
   * @brief calculate geodetic uding gnss and dvl odometry
   * @details Acquisitionn the dvl odometry.
   */
  void _calc_global_pose(const localization_msgs::msg::Odometry::SharedPtr odom_);

  /**
   * @brief If all data is updated, Publish odometry.
   * @note If dvl, imu and depth are not updated, do not publish.
   */
  void publisher();

  /**
   * @brief Call reset func
   * @details Execute a reset function when requested by the client.
   */
  void reset_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    const std_srvs::srv::Trigger::Response::SharedPtr response);

  /**
   * @brief Check if the server is running
   */
  bool _is_server_active(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_);

  /**
   * @brief Send request to server
   */
  FutureAndRequestId _reset_request(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_);

  /**
   * @brief Wait response and Check status
   */
  bool _reset_response(
    FutureAndRequestId future, std::chrono::duration<long, std::ratio<1, 1000>> dulation,
    std::string service_name);

public:
  /**
   * @brief Set topic, service
   * @details Instantiate topic(pub-sub), and service(server, client)
   * @note The calculation cycle depends on the cycle of the topic sent from the depth, imu, and dvl driver.
   */
  explicit Localization(const rclcpp::NodeOptions & options);

  /**
   * @brief Reset each odometry and imu transformed node
   */
  int reset();
};

}  // namespace localization

#endif
