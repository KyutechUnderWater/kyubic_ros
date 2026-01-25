/**
 * @file dvl_driver.hpp
 * @brief DVL driver Node Definition
 * @author R.Ohnishi
 * @date 2024/10/30
 *
 * @details DVL(Path Finder) のPD0/PD5データを取得して，Topicを流す
 *****************************************************************/

#ifndef _DVL_DRIVER_HPP
#define _DVL_DRIVER_HPP

#include <driver_msgs/msg/dvl.hpp>
#include <driver_msgs/srv/command.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dvl_driver/path_finder.hpp"
#include "timer/timeout.hpp"

/**
 * @namespace driver::dvl_driver
 * @brief Namespace for DVL driver components
 */
namespace driver::dvl_driver
{

/**
 * @class DVLDriver
 * @brief ROS 2 Node for interfacing with Teledyne Pathfinder DVL
 */
class DVLDriver : public rclcpp::Node
{
public:
  /**
   * @brief Constructor. Establishes connection and initializes publishers/services.
   */
  explicit DVLDriver();

  /**
   * @brief Destructor. Sends break command to stop DVL output.
   */
  ~DVLDriver();

private:
  std::string address;
  int listener_port;
  int sender_port;
  int64_t timeout_ms;
  bool turning;

  bool command_mode = false;
  bool break_cmd = false;
  std::shared_ptr<timer::Timeout> timeout_;

  // DVL Interface
  std::shared_ptr<path_finder::Sender> sender_;
  std::shared_ptr<path_finder::Listener> listener_;

  // ROS Interface
  rclcpp::Publisher<driver_msgs::msg::DVL>::SharedPtr pub_;
  rclcpp::Service<driver_msgs::srv::Command>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Initialize DVL connection (Send break, check response)
   * @return true if initialization successful
   */
  bool setup();

  /**
   * @brief Perform one update cycle (Ping -> Listen)
   * @return true if data received successfully
   */
  bool _update();

  /**
   * @brief Timer callback to process DVL data and publish messages
   */
  void update();

  /**
   * @brief Convert raw velocity data to ROS message format
   * @tparam T Array type containing velocity data (int16_t[4])
   * @param raw_vel Raw velocity array [vx, vy, vz, error] in mm/s
   * @param msg Destination DVL message to update
   */
  template <typename T>
  void _convertVelocity(const T & raw_vel, driver_msgs::msg::DVL & msg);

  /**
   * @brief Extract data from PD0 ensemble and populate ROS message
   * @details Parses system config, leak status, velocity, range, and beam status.
   * Also handles debug logging if 'turning' flag is active.
   * @param data Shared pointer to the parsed PD0 data structure
   * @param msg Reference to the DVL message to be populated
   */
  void _extractPd0Data(
    const std::shared_ptr<path_finder::pd0::Pd0Ensemble> & data, driver_msgs::msg::DVL & msg);

  /**
   * @brief Extract data from PD5 ensemble and populate ROS message
   * @details Parses system config, leak status (from BIT), velocity, range, and beam status.
   * @param data Shared pointer to the parsed PD5 data structure
   * @param msg Reference to the DVL message to be populated
   */
  void _extractPd5Data(
    const std::shared_ptr<path_finder::pd5::Pd5Ensemble> & data, driver_msgs::msg::DVL & msg);

  /**
   * @brief Helper to calculate average altitude from beam ranges
   * @param ranges Array of 4 beam ranges in meters
   * @return Average altitude in meters (0.0 if no valid beams)
   */
  double _calculate_average_altitude(const float ranges[4]);

  /**
   * @brief Service callback to send raw commands to DVL
   * @param request Service request containing command string
   * @param response Service response containing DVL output
   */
  void sendCommandCallback(
    const driver_msgs::srv::Command::Request::SharedPtr request,
    driver_msgs::srv::Command::Response::SharedPtr response);
};

}  // namespace driver::dvl_driver

#endif  // _DVL_DRIVER_HPP
