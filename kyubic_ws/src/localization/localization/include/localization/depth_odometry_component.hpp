/**
 * @file depth_odometry.hpp
 * @brief Calculating velocity by differentiating depth data
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details 深度センサのデータを微分してz-axisの速度を算出
 **********************************************************/

#ifndef _DEPTH_ODOMETRY_HPP
#define _DEPTH_ODOMETRY_HPP

#include <rclcpp/rclcpp.hpp>

#include <driver_msgs/msg/depth.hpp>
#include <localization_msgs/msg/odometry.hpp>

namespace localization
{

class DepthOdometry : public rclcpp::Node
{
private:
  rclcpp::Publisher<localization_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<driver_msgs::msg::Depth>::SharedPtr sub_;

  rclcpp::Time pre_time;
  double pre_pos_z = 0.0;

  void _update_callback(const driver_msgs::msg::Depth::UniquePtr msg);

public:
  explicit DepthOdometry(const rclcpp::NodeOptions & options);

  void reset();
};

}  // namespace localization

#endif  // !_DEPTH_ODOMETRY_HPP
