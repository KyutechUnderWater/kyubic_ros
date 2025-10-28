/**
 * @file imu_transform_component.hpp
 * @brief convert from imu frame to map frame
 * @author R.Ohnishi
 * @date 2025/05/30
 *
 * @details imuの座標系をワールド座標系に変換する
 ***********************************************/

#include <rclcpp/rclcpp.hpp>
#include <transform/transform.hpp>

#include <driver_msgs/msg/imu.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

/**
 * @namespace localization
 * @brief localization
 */
namespace localization
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

  Transform3D::Rotation rot_;

  uint8_t status_ = localization_msgs::msg::Status::ERROR;
  Eigen::Vector3d gyro_;
  Eigen::Vector3d orient_;
  Eigen::Vector3d pre_euler;
  Eigen::Vector3d offset_orient_;
  Eigen::Quaterniond qtn_;
  Eigen::Quaterniond offset_qtn_;

  /**
   * @brief Update imu data
   * @details After getting the imu data, transform from imu coordinate system to map coordinate system
   * @note The calculation cycle depends on the cycle of the topic sent from the imu driver.
   */
  void update_callback(const driver_msgs::msg::IMU::UniquePtr msg);

  /**
   * @brief Calculate euler angle
   * @details After getting the imu data, transform from imu coordinate system to map coordinate system
   */
  driver_msgs::msg::IMU::UniquePtr calc_euler(driver_msgs::msg::IMU::UniquePtr msg);

  /**
   * @brief Calculate quaternion
   * @details After getting the imu data, transform from imu coordinate system to map coordinate system
   */
  driver_msgs::msg::IMU::UniquePtr calc_quartenion(driver_msgs::msg::IMU::UniquePtr msg);

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
   * @return If success is true, otherwise is false.
   */
  bool reset();
};

}  // namespace localization
