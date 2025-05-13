/**
 * @file localization.hpp
 * @brief localization using dvl and depth
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details DVLとDepthセンサを用いた自己位置推定
 ****************************************************/

#ifndef _LOCALIZATIN_HPP
#define _LOCALIZATIN_HPP

#include <rclcpp/rclcpp.hpp>

#include <localization_msgs/msg/odometry.hpp>

namespace localization
{

class Localization : rclcpp::Node
{
private:
  rclcpp::Publisher<localization_msgs::msg::Odometry>::SharedPtr pub_;

public:
  explicit Localization();
};

}  // namespace localization

#endif
