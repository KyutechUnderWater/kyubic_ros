/**
 * @file imu_transform_component.hpp
 * @brief convert from imu frame to map frame
 * @author R.Ohnishi
 * @date 2025/05/30
 *
 * @details imuの座標系をワールド座標系に変換する
 ***********************************************/

#include <rclcpp/rclcpp.hpp>

#include <driver_msgs/msg/imu.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>

namespace localization
{

class IMUTransform : public rclcpp::Node
{
private:
  rclcpp::Publisher<localization_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<driver_msgs::msg::IMU>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;

  std::array<double, 3> offset_angle;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  void update_callback(const driver_msgs::msg::IMU::UniquePtr msg);
  void reset_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    const std_srvs::srv::Trigger::Response::SharedPtr response);

public:
  explicit IMUTransform(const rclcpp::NodeOptions & options);

  void reset();
};

}  // namespace localization
