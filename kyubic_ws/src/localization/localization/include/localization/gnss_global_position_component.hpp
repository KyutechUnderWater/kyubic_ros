/**
 * @file gnss_global_position_component.hpp
 * @brief calculating global position
 * @author R.Ohnishi
 * @date 2025/07/30
 *
 * @details gnssの座標系をワールド座標系に変換する
 ***********************************************/

#include <driver_msgs/msg/gnss.hpp>
#include <driver_msgs/msg/imu.hpp>
#include <geodetic_converter/geodetic_converter.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

/**
 * @namespace localization::gnss
 * @brief localization
 */
namespace localization::gnss
{

/**
 * @brief Calculate global pose from gnss
 */
class GnssGlobalPosition : public rclcpp::Node
{
private:
  uint8_t coord_system_id;
  double azimuth_offset;
  rclcpp::CallbackGroup::SharedPtr gnss_cb_group_;
  rclcpp::Publisher<localization_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<driver_msgs::msg::Gnss>::SharedPtr sub_gnss_;
  rclcpp::Subscription<driver_msgs::msg::IMU>::SharedPtr sub_imu_raw_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;

  std::shared_ptr<common::GeodeticConverter> geo_converter_;
  std::shared_ptr<driver_msgs::msg::Gnss> gnss_msg_;
  std::shared_ptr<driver_msgs::msg::IMU> imu_raw_msg_;

  common::Geodetic ref_geodetic;
  common::PlaneXY ref_plane;
  double azimuth;

  bool reset_flag = false;

  std::atomic<bool> gnss_updated{false};
  std::atomic<bool> imu_updated{false};
  std::mutex mtx_gnss_;
  std::mutex mtx_imu_;

  /**
   * @brief Update global pose
   * @details gnss and dvl data are used to calculate global pose
   */
  void update_callback(const localization_msgs::msg::Odometry::UniquePtr msg);

  /**
   * @brief Update gnss data
   * @details Acquisition the gnss data.
   */
  void gnss_callback(const driver_msgs::msg::Gnss::UniquePtr msg);

  /**
   * @brief Update imu data
   * @details Acquisition the imu data.
   */
  void imu_raw_callback(const driver_msgs::msg::IMU::UniquePtr msg);

  /**
   * @brief Wait for updated data
   * @param updated updated flag
   * @param mtx mutex
   * @param timeout_count timeout count (1count is waiting 100ms)
   */
  bool _waiting_update(std::atomic<bool> & updated, uint8_t timeout_count = 15);

  /**
   * @brief Call reset func
   * @details Determine the reference point when requested by the client.
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
  explicit GnssGlobalPosition(const rclcpp::NodeOptions & options);
};

}  // namespace localization::gnss
