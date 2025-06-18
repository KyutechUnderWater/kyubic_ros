/**
 * @file localization.hpp
 * @brief localization using dvl and depth
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details DVLとDepthセンサを用いた自己位置推定
 ****************************************************/

#ifndef _LOCALIZATIN_COMPONENT_HPP
#define _LOCALIZATIN_COMPONENT_HPP

#include <rclcpp/rclcpp.hpp>

#include <localization_msgs/msg/odometry.hpp>

namespace localization
{

class Localization : public rclcpp::Node
{
private:
  rclcpp::Publisher<localization_msgs::msg::Odometry>::SharedPtr pub_;

  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_depth_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_imu_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_dvl_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<localization_msgs::msg::Odometry> odom_msg_;

  uint8_t all_updated = 0b11111000;

  void _depth_callback(const localization_msgs::msg::Odometry::UniquePtr msg);
  void _imu_callback(const localization_msgs::msg::Odometry::UniquePtr msg);
  void _dvl_callback(const localization_msgs::msg::Odometry::UniquePtr msg);

  void _merge();

public:
  explicit Localization(const rclcpp::NodeOptions & options);
};

}  // namespace localization

#endif
