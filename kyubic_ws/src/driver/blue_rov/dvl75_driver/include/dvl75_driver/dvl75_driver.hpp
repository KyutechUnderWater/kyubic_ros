/**
 * @file dvl75_driver.hpp
 * @brief ROS 2 node declaration for the Cerulean Sonar DVL-75.
 */

#ifndef DVL75_DRIVER__DVL75_DRIVER_HPP_
#define DVL75_DRIVER__DVL75_DRIVER_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "custom_socket/udp.hpp"
#include "dvl75_driver/dvl75_parser.hpp"
#include "rclcpp/rclcpp.hpp"

#include "blue_rov_msgs/msg/dvl75.hpp"
#include "driver_msgs/msg/dvl.hpp"
#include "driver_msgs/srv/command.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace dvl75_driver
{

/**
 * @brief Convert DVL-75 UDP measurements into ROS 2 DVL messages.
 *
 * The node accepts packets only from the configured DVL address. It caches the
 * latest DVEXT sentence and combines it with each DVPDL sentence to publish
 * the common DVL message and the DVL-75-specific message.
 */
class Dvl75Driver : public rclcpp::Node
{
public:
  /**
   * @brief Construct the DVL-75 node and start UDP reception.
   *
   * @param options ROS 2 node options, including parameter overrides.
   * @throws std::invalid_argument When a network, validation, or axis parameter is invalid.
   * @throws std::runtime_error When the UDP socket cannot be created or bound.
   */
  explicit Dvl75Driver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Stop UDP reception and close the socket.
   */
  ~Dvl75Driver() override;

private:
  using SteadyTime = std::chrono::steady_clock::time_point;
  using AxisSigns = std::array<double, 3>;

  void validate_configuration() const;
  static AxisSigns parse_axis_sign(const std::vector<double> & signs, const char * parameter_name);
  void create_udp_socket();
  void apply_configuration();
  void send_command(const std::string & command);
  void receive_loop();
  void process_packet(
    const std::string & packet, const SteadyTime & received_at,
    const builtin_interfaces::msg::Time & received_stamp);
  void publish_dvpdl(
    const Dvpdl & dvpdl, const std::optional<Dvext> & dvext,
    const std::array<double, 3> & position_delta,
    const builtin_interfaces::msg::Time & received_stamp);
  void publish_dvl75(
    const std::optional<Dvext> & dvext, const Dvpdl & dvpdl,
    const std::array<double, 3> & position_delta, const std::array<double, 3> & angle_delta,
    const builtin_interfaces::msg::Time & received_stamp);
  void publish_error_dvl(const builtin_interfaces::msg::Time & stamp);
  void timeout_callback();
  void report_tracking_status(
    std::uint8_t status, std::size_t locked_beam_count, std::uint8_t confidence);

  static AxisSigns apply_axis_sign(const std::array<double, 3> & values, const AxisSigns & signs);
  static bool disables_required_output(const std::string & command);

  std::string dvl_address_;
  int dvl_command_port_{};
  std::string bind_address_;
  int listen_port_{};
  std::string frame_id_;
  double timeout_s_{};
  double dvext_timeout_s_{};
  bool validate_checksum_{};
  int minimum_confidence_{};
  int minimum_locked_beams_{};
  AxisSigns position_axis_sign_{};
  AxisSigns angle_axis_sign_{};
  bool apply_device_config_{};
  std::string device_host_address_;
  std::vector<std::string> startup_commands_;

  std::unique_ptr<common::UdpSocket> socket_;
  std::atomic_bool stop_requested_{false};
  std::thread receiver_thread_;

  mutable std::mutex data_mutex_;
  std::optional<Dvext> last_dvext_;
  SteadyTime last_dvext_time_{};
  SteadyTime last_dvpdl_time_{};
  bool timeout_reported_{false};
  std::optional<std::uint8_t> last_tracking_status_;
  std::optional<SteadyTime> dvpdl_enable_requested_at_;

  rclcpp::Publisher<driver_msgs::msg::DVL>::SharedPtr dvl_publisher_;
  rclcpp::Publisher<blue_rov_msgs::msg::DVL75>::SharedPtr dvl75_publisher_;
  rclcpp::Service<driver_msgs::srv::Command>::SharedPtr command_service_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

}  // namespace dvl75_driver

#endif  // DVL75_DRIVER__DVL75_DRIVER_HPP_
