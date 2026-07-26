/**
 * @file dvl75_driver.hpp
 * @brief ROS 2 node declaration for the Cerulean Sonar DVL-75.
 */

#ifndef DVL75_DRIVER__DVL75_DRIVER_HPP_
#define DVL75_DRIVER__DVL75_DRIVER_HPP_

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

#include "blue_rov_msgs/msg/dvl75.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "custom_socket/udp.hpp"
#include "driver_msgs/msg/dvl.hpp"
#include "driver_msgs/srv/command.hpp"
#include "dvl75_driver/dvl75_parser.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @namespace dvl75_driver
 * @brief ROS 2 driver and parser support for the Cerulean Sonar DVL-75.
 */
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
  /** @brief Monotonic timestamp used for packet freshness and timeout checks. */
  using SteadyTime = std::chrono::steady_clock::time_point;

  /** @brief Per-axis multipliers applied to DVL vectors. */
  using AxisSigns = std::array<double, 3>;

  /**
   * @brief Validate network, timeout, tracking, and threshold parameters.
   * @throws std::invalid_argument If a parameter is outside its supported range.
   */
  void validate_configuration() const;

  /**
   * @brief Convert a ROS axis-multiplier parameter into the fixed internal representation.
   * @param signs Three finite, non-zero axis multipliers.
   * @param parameter_name Parameter name included in validation errors.
   * @return Validated X, Y, and Z multipliers.
   * @throws std::invalid_argument If the input does not contain three valid values.
   */
  static AxisSigns parse_axis_sign(const std::vector<double> & signs, const char * parameter_name);

  /**
   * @brief Create, configure, and bind the UDP socket used by the driver.
   * @throws std::runtime_error If the socket cannot bind to the configured endpoint.
   */
  void create_udp_socket();

  /**
   * @brief Send the provisioning command sequence to the DVL.
   * @throws std::exception If a command is invalid or cannot be sent.
   */
  void apply_configuration();

  /**
   * @brief Validate and send one newline-terminated DVL command.
   * @param command Command text without a line terminator.
   * @throws std::invalid_argument If the command is empty, multiline, or disables required output.
   * @throws std::runtime_error If the UDP datagram cannot be sent completely.
   */
  void send_command(const std::string & command);

  /** @brief Receive DVL UDP datagrams until node shutdown is requested. */
  void receive_loop();

  /**
   * @brief Parse and dispatch all NMEA sentences contained in one UDP datagram.
   * @param packet Raw UDP payload converted to an ASCII string.
   * @param received_at Monotonic host reception time used for freshness checks.
   * @param received_stamp ROS reception time assigned to published message headers.
   */
  void process_packet(
    const std::string & packet, const SteadyTime & received_at,
    const builtin_interfaces::msg::Time & received_stamp);

  /**
   * @brief Publish a common DVL message derived from one DVPDL measurement.
   * @param dvpdl Parsed incremental position measurement.
   * @param dvext Fresh DVEXT tracking data, if available.
   * @param position_delta Position delta after configured axis conversion.
   * @param received_stamp ROS reception time assigned to the message header.
   */
  void publish_dvpdl(
    const Dvpdl & dvpdl, const std::optional<Dvext> & dvext,
    const std::array<double, 3> & position_delta,
    const builtin_interfaces::msg::Time & received_stamp);

  /**
   * @brief Publish the complete DVL-75-specific measurement snapshot.
   * @param dvext Fresh DVEXT tracking data, if available.
   * @param dvpdl Parsed incremental position measurement.
   * @param position_delta Position delta after configured axis conversion.
   * @param angle_delta Angle delta after configured axis conversion.
   * @param received_stamp ROS reception time assigned to the message header.
   */
  void publish_dvl75(
    const std::optional<Dvext> & dvext, const Dvpdl & dvpdl,
    const std::array<double, 3> & position_delta, const std::array<double, 3> & angle_delta,
    const builtin_interfaces::msg::Time & received_stamp);

  /**
   * @brief Publish an invalid common DVL measurement.
   * @param stamp ROS time assigned to the error message header.
   */
  void publish_error_dvl(const builtin_interfaces::msg::Time & stamp);

  /** @brief Detect DVPDL reception timeouts and publish one error transition. */
  void timeout_callback();

  /**
   * @brief Log a tracking-quality transition without repeating an unchanged status.
   * @param status Common status identifier for the current measurement.
   * @param locked_beam_count Number of beams currently locked on the target surface.
   * @param confidence DVL confidence in the range 0 to 100.
   */
  void report_tracking_status(
    std::uint8_t status, std::size_t locked_beam_count, std::uint8_t confidence);

  /**
   * @brief Apply configured axis multipliers to a three-dimensional vector.
   * @param values Source X, Y, and Z values.
   * @param signs X, Y, and Z multipliers.
   * @return Converted vector.
   */
  static AxisSigns apply_axis_sign(const std::array<double, 3> & values, const AxisSigns & signs);

  /**
   * @brief Check whether a command disables a measurement required by this driver.
   * @param command Normalized or user-provided DVL command.
   * @return true when the command disables DVEXT or DVPDL output.
   */
  static bool disables_required_output(const std::string & command);

  std::string dvl_address_;          ///< Expected DVL IPv4 address.
  int dvl_command_port_{};           ///< DVL UDP command port.
  std::string bind_address_;         ///< Local IPv4 address used for reception.
  int listen_port_{};                ///< Local UDP measurement port.
  std::string frame_id_;             ///< Frame assigned to published measurements.
  double timeout_s_{};               ///< Maximum permitted DVPDL reception gap.
  double dvext_timeout_s_{};         ///< Maximum age of cached DVEXT data.
  bool validate_checksum_{};         ///< Whether NMEA checksums are mandatory.
  int minimum_confidence_{};         ///< Minimum accepted DVPDL confidence.
  int minimum_locked_beams_{};       ///< Minimum accepted locked-beam count.
  AxisSigns position_axis_sign_{};   ///< Position-delta axis multipliers.
  AxisSigns angle_axis_sign_{};      ///< Angle-delta axis multipliers.
  bool apply_device_config_{};       ///< Whether provisioning commands are sent at startup.
  std::string device_host_address_;  ///< Optional HOST-ADDRESS command value.
  std::vector<std::string> startup_commands_;  ///< Additional provisioning commands.

  std::unique_ptr<common::UdpSocket> socket_;  ///< UDP transport owned by this node.
  std::atomic_bool stop_requested_{false};     ///< Receiver-thread shutdown request.
  std::thread receiver_thread_;                ///< Background UDP receiver.

  mutable std::mutex data_mutex_;  ///< Protects cached measurements and status.
  /** @brief Most recently received DVEXT measurement. */
  std::optional<Dvext> last_dvext_;
  SteadyTime last_dvext_time_{};  ///< Reception time of the cached DVEXT.
  SteadyTime last_dvpdl_time_{};  ///< Reception time of the latest DVPDL.
  /** @brief Whether the current timeout was published. */
  bool timeout_reported_{false};
  std::optional<std::uint8_t> last_tracking_status_;     ///< Last logged tracking status.
  std::optional<SteadyTime> dvpdl_enable_requested_at_;  ///< SEND-DVPDL ON send time.

  rclcpp::Publisher<driver_msgs::msg::DVL>::SharedPtr dvl_publisher_;        ///< Common DVL output.
  rclcpp::Publisher<blue_rov_msgs::msg::DVL75>::SharedPtr dvl75_publisher_;  ///< Raw output.
  rclcpp::Service<driver_msgs::srv::Command>::SharedPtr command_service_;    ///< Command service.
  rclcpp::TimerBase::SharedPtr timeout_timer_;  ///< Periodic receive-timeout checker.
};

}  // namespace dvl75_driver

#endif  // DVL75_DRIVER__DVL75_DRIVER_HPP_
