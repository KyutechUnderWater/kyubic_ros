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

namespace localization
{

class IMUTransform : public rclcpp::Node
{
private:
  rclcpp::Publisher<localization_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<driver_msgs::msg::IMU>::SharedPtr sub_;

  void _transform_callback(const driver_msgs::msg::IMU::UniquePtr msg);

public:
  explicit IMUTransform(const rclcpp::NodeOptions & options);
};

}  // namespace localization
