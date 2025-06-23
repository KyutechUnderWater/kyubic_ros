/**
 * @file depth_odometry.hpp
 * @brief Calculating velocity by differentiating depth data
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details 深度センサのデータを微分してz-axisの速度を算出
 * @note 計算周期は depth driver から送られてくる Topic に依存
 * @note 正常なデータを受信したときのみ，計算して Publish する
 *
 ***************************************************************/

#ifndef _DEPTH_ODOMETRY_HPP
#define _DEPTH_ODOMETRY_HPP

#include <rclcpp/rclcpp.hpp>

#include <driver_msgs/msg/depth.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

/**
 * @namespace localization
 * @brief localization
 */
namespace localization
{

/** @brief Sample size of simple moving average */
const int DEPTH_SMA_SUMPLE_NUM = 16;

/**
 * @brief Depth odometry class
 */
class DepthOdometry : public rclcpp::Node
{
private:
  rclcpp::Publisher<localization_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<driver_msgs::msg::Depth>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;

  rclcpp::Time pre_time;
  double pre_pos_z = 0.0;

  int8_t idx = 0;
  std::array<double, DEPTH_SMA_SUMPLE_NUM> pos_z_list;  /// for simple moving average

  /**
   * @brief Update z-axis velocity
   * @details After getting the depth data, the velocity is calculated based on the previous value and send to topic.
   * @note The calculation cycle depends on the cycle of the topic sent from the depth driver.
   * @note If no data comes in or the data is invalid, no calculation is performed.
   */
  void update_callback(const driver_msgs::msg::Depth::UniquePtr msg);

  /**
   * @brief Call reset func
   * @details Execute a reset function when requested by the client.
   */
  void reset_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    const std_srvs::srv::Trigger::Response::SharedPtr response);

public:
  /**
   * @brief Set topic and Reset timer
   * @details Instantiate topic(pub-sub), Call to reset function
   * @note The calculation cycle depends on the cycle of the topic sent from the depth driver.
   */
  explicit DepthOdometry(const rclcpp::NodeOptions & options);

  /**
   * @brief Set previous depth data to 0, and Reset timer
   */
  void reset();
};

}  // namespace localization

#endif  // !_DEPTH_ODOMETRY_HPP
