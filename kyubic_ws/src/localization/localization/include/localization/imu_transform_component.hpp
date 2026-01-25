/**
 * @file imu_transform_component.hpp
 * @brief convert from imu frame to map frame
 * @author R.Ohnishi
 * @date 2025/05/30
 *
 * @details imuの座標系をワールド座標系に変換する
 ***********************************************/

#include <array>
#include <driver_msgs/msg/imu.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

/**
 * @namespace localization::imu
 * @brief localization
 */
namespace localization::imu
{

/**
 * @brief IMU transform class
 */
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

  /**
   * @brief Update imu data
   * @details After getting the imu data, transform from imu coordinate system to map coordinate system
   * @note The calculation cycle depends on the cycle of the topic sent from the imu driver.
   */
  void update_callback(const driver_msgs::msg::IMU::UniquePtr msg);

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
   * @note The calculation cycle depends on the cycle of the topic sent from the imu driver.
   */
  explicit IMUTransform(const rclcpp::NodeOptions & options);

  /**
   * @brief Set offset_angle
   */
  void reset();
};

}  // namespace localization::imu
