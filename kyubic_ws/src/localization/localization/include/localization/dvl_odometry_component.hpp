/**
 * @file dvl_odometry.hpp
 * @brief Calculating position by accumulating DVL velocity
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details DVLで取得した速度を累積して，位置を算出
 * @note 計算周期は dvl driver から送られてくる Topic に依存
 * @note 正常なデータを受信したときのみ，計算して Publish する
 *********************************************************/

#ifndef _DVL_ODOMETRY_HPP
#define _DVL_ODOMETRY_HPP

#include <driver_msgs/msg/dvl.hpp>
#include <driver_msgs/msg/imu.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "std_srvs/srv/trigger.hpp"

/**
 * @namespace localization
 * @brief localization
 */
namespace localization
{

/**
 * @brief DVL odometry class
 */
class DVLOdometry : public rclcpp::Node
{
private:
  rclcpp::Publisher<localization_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_imu_;
  rclcpp::Subscription<driver_msgs::msg::DVL>::SharedPtr sub_dvl_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;

  rclcpp::Time pre_time;

  std::shared_ptr<localization_msgs::msg::Odometry> imu_msg_;

  double pos_x = 0.0;
  double pos_y = 0.0;

  /**
   * @brief Update position
   * @details After getting the dvl data, the position is calculated based on the previous value and send to topic.
   * @note The calculation cycle depends on the cycle of the topic sent from the depth driver.
   * @note If no data comes in or the data is invalid, no calculation is performed.
   */
  void update_callback(const driver_msgs::msg::DVL::UniquePtr msg);

  /**
   * @brief Update imu data
   * @details Acquisition the imu data.
   * @note The calculation cycle depends on the cycle of the topic sent from the imu driver.
   */
  void update_imu_callback(const localization_msgs::msg::Odometry::UniquePtr msg);

  /**
   * @brief Call reset func
   * @details Execute a reset function when requested by the client.
   */
  void reset_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    const std_srvs::srv::Trigger::Response::SharedPtr response);

public:
  /**
   * @brief Set topic and Reset
   * @details Instantiate topic(pub-sub), Call to reset function.
   * @note The calculation cycle depends on the cycle of the topic sent from the dvl driver.
   */
  explicit DVLOdometry(const rclcpp::NodeOptions & options);

  /**
   * @brief Set position data to 0, and Reset timer
   */
  void reset();
};
}  // namespace localization

#endif
