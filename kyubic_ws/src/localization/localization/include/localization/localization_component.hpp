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
#include <std_srvs/srv/trigger.hpp>

namespace localization
{

using FutureAndRequestId = rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture;

class Localization : public rclcpp::Node
{
private:
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::Publisher<localization_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_depth_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_imu_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_dvl_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_depth_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_imu_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_dvl_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<localization_msgs::msg::Odometry> odom_msg_;

  uint8_t all_updated = 0b11111000;

  void depth_callback(const localization_msgs::msg::Odometry::UniquePtr msg);
  void imu_callback(const localization_msgs::msg::Odometry::UniquePtr msg);
  void dvl_callback(const localization_msgs::msg::Odometry::UniquePtr msg);
  void reset_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    const std_srvs::srv::Trigger::Response::SharedPtr response);

  void merge();

  bool _is_server_active(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_);

  FutureAndRequestId _reset_request(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_);

  bool _reset_response(
    FutureAndRequestId future, std::chrono::duration<long, std::ratio<1, 1000>> dulation,
    std::string service_name);

public:
  explicit Localization(const rclcpp::NodeOptions & options);

  int reset();
};

}  // namespace localization

#endif
