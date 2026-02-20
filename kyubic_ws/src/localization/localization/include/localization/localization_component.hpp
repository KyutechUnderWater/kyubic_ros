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

#include <localization_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

/**
 * @namespace localization
 * @brief localization
 */
namespace localization
{

const std::array<std::array<double, 3>, 2> ANCHOR_POINTS_DMS[4] = {
  {{{33, 59, 14.8}, {132, 12, 5.4}}},  // I系: 33°00′00″N, 129°30′00″E
  {{{33, 59, 14.6}, {132, 12, 6.4}}},  // II系: 33°00′00″N, 131°00′00″E
  {{{33, 59, 15.2}, {132, 12, 6.4}}},  // III系: 36°00′00″N, 132°10′30″E
  {{{33, 59, 15.5}, {132, 12, 5.7}}},  // IV系: 33°00′00″N, 133°00′00″E
};

/** Create alias for namespace */
using FutureAndRequestId = rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture;

/**
 * @brief Localization class
 */
class Localization : public rclcpp::Node
{
private:
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::Publisher<localization_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_depth_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_imu_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_dvl_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_gnss_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_depth_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_imu_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_dvl_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_gnss_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<localization_msgs::msg::Odometry> odom_depth_;
  std::shared_ptr<localization_msgs::msg::Odometry> odom_msg_;

  bool depth_enable_ = false;
  bool imu_enable_ = false;
  bool dvl_enable_ = false;
  bool gnss_enable_ = false;
  uint8_t enabled_sensors_ = 0;
  uint8_t updated_ = 0;

  /**
   * @brief Update depth odometry
   * @details Acquisition the depth odometry.
   */
  void depth_callback(const localization_msgs::msg::Odometry::UniquePtr msg);

  /**
   * @brief Update imu transformed
   * @details Acquisition the imu transformed data.
   */
  void imu_callback(const localization_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Update dvl odometry
   * @details Acquisitionn the dvl odometry.
   */
  void dvl_callback(const localization_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Update global position
   * @details Acquisitionn the global position.
   */
  void gnss_callback(const localization_msgs::msg::Odometry::SharedPtr msg);

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
