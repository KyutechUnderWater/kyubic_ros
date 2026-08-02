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

#include <tf2/LinearMath/Vector3.h>

#include <driver_msgs/msg/dvl.hpp>
#include <driver_msgs/msg/imu.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "std_srvs/srv/trigger.hpp"

/**
 * @namespace localization::dvl
 * @brief localization
 */
namespace localization::dvl
{

inline constexpr double DEGREE_TO_RADIAN = std::numbers::pi / 180.0;

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

  std::array<float, 3> offset;
  std::shared_ptr<localization_msgs::msg::Odometry> imu_msg_;

  double pos_x = 0.0;
  double pos_y = 0.0;
  // ERROR分岐(velocity_valid==false)でもzero値を送らないための直前値保持。
  // pos_x/pos_yは元々累積値として保持されていたが、publishする際にERROR分岐で
  // セットし忘れていた(Part A参照)。altitude/velocityは新規に保持する。
  // pos_x/pos_yは reset() の0初期化がdead-reckoning原点として正当な値だが、
  // altitude/velocityは「一度も有効値を受信していない」場合に0を保持値と混同すると
  // Part Aと同じ問題が起動直後の数秒間だけ再発するため、has_valid_measurement_で区別する。
  double last_altitude_ = 0.0;
  tf2::Vector3 last_velocity_world_{0.0, 0.0, 0.0};
  bool has_valid_measurement_ = false;
  /// velocity_valid==false でも一度でも有効計測があれば WARNING で推定を継続する。
  bool degraded_coast_on_invalid_dvl_ = true;

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
}  // namespace localization::dvl

#endif
