/**
 * @file dvl75_driver.cpp
 * @brief ROS 2 UDP node implementation for the Cerulean Sonar DVL-75.
 */

#include "dvl75_driver/dvl75_driver.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include "common_msgs/msg/status.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace dvl75_driver
{
namespace
{

constexpr int kDefaultDvlPort = 50000;
constexpr char kDefaultDvlAddress[] = "192.168.2.3";
constexpr int kPacketBufferSize = 4096;
constexpr std::chrono::milliseconds kReceiveTimeout{200};

std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1U);
}

void set_vector(geometry_msgs::msg::Vector3 & target, const std::array<double, 3> & values)
{
  target.x = values[0];
  target.y = values[1];
  target.z = values[2];
}

template <typename MessageT>
void set_message_header(
  MessageT & message, const builtin_interfaces::msg::Time & stamp, const std::string & frame_id)
{
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
}

}  // namespace

Dvl75Driver::Dvl75Driver(const rclcpp::NodeOptions & options) : Node("dvl75_driver", options)
{
  dvl_address_ = declare_parameter<std::string>("dvl_address", kDefaultDvlAddress);
  dvl_command_port_ = declare_parameter<int>("dvl_command_port", kDefaultDvlPort);
  bind_address_ = declare_parameter<std::string>("bind_address", "0.0.0.0");
  listen_port_ = declare_parameter<int>("listen_port", 27000);
  frame_id_ = declare_parameter<std::string>("frame_id", "dvl75_link");
  timeout_s_ = declare_parameter<double>("timeout_s", 1.0);
  dvext_timeout_s_ = declare_parameter<double>("dvext_timeout_s", 1.0);
  validate_checksum_ = declare_parameter<bool>("validate_checksum", true);
  minimum_confidence_ = declare_parameter<int>("minimum_confidence", 50);
  minimum_locked_beams_ = declare_parameter<int>("minimum_locked_beams", 3);
  const auto position_axis_sign =
    declare_parameter<std::vector<double>>("position_axis_sign", {1.0, 1.0, 1.0});
  const auto angle_axis_sign =
    declare_parameter<std::vector<double>>("angle_axis_sign", {1.0, 1.0, 1.0});
  apply_device_config_ = declare_parameter<bool>("apply_device_config", false);
  device_host_address_ = declare_parameter<std::string>("device_host_address", "");
  startup_commands_ =
    declare_parameter<std::vector<std::string>>("startup_commands", std::vector<std::string>{});
  validate_configuration();
  position_axis_sign_ = parse_axis_sign(position_axis_sign, "position_axis_sign");
  angle_axis_sign_ = parse_axis_sign(angle_axis_sign, "angle_axis_sign");

  const auto now = std::chrono::steady_clock::now();
  last_dvpdl_time_ = now;
  last_dvext_time_ = now;

  dvl_publisher_ = create_publisher<driver_msgs::msg::DVL>("dvl", 10);
  dvl75_publisher_ = create_publisher<blue_rov_msgs::msg::DVL75>("dvl75", 10);
  command_service_ = create_service<driver_msgs::srv::Command>(
    "command", [this](
                 const std::shared_ptr<driver_msgs::srv::Command::Request> request,
                 std::shared_ptr<driver_msgs::srv::Command::Response> response) {
      try {
        send_command(request->command);
        response->output = "Command sent; inspect DVL output for any device response.";
      } catch (const std::exception & error) {
        response->output = std::string("Command rejected or send failed: ") + error.what();
      }
    });
  timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(200), std::bind(&Dvl75Driver::timeout_callback, this));

  create_udp_socket();
  RCLCPP_INFO(
    get_logger(),
    "DVL-75 endpoints: receive=%s:%d, dvl_address=%s, command_port=%d, "
    "apply_device_config=%s",
    bind_address_.c_str(), listen_port_, dvl_address_.c_str(), dvl_command_port_,
    apply_device_config_ ? "true" : "false");
  if (apply_device_config_) {
    apply_configuration();
  } else {
    RCLCPP_INFO(
      get_logger(), "DVL persistent configuration is disabled; expecting DVEXT and DVPDL output.");
  }

  receiver_thread_ = std::thread(&Dvl75Driver::receive_loop, this);
}

Dvl75Driver::~Dvl75Driver()
{
  stop_requested_.store(true);
  if (receiver_thread_.joinable()) {
    receiver_thread_.join();
  }
  socket_.reset();
}

void Dvl75Driver::validate_configuration() const
{
  if (listen_port_ < 1 || listen_port_ > 65535) {
    throw std::invalid_argument("listen_port must be in the range 1..65535");
  }
  if (dvl_command_port_ < 1 || dvl_command_port_ > 65535) {
    throw std::invalid_argument("dvl_command_port must be in the range 1..65535");
  }
  if (timeout_s_ <= 0.0 || dvext_timeout_s_ <= 0.0) {
    throw std::invalid_argument("timeout_s and dvext_timeout_s must be positive");
  }
  if (minimum_confidence_ < 0 || minimum_confidence_ > 100) {
    throw std::invalid_argument("minimum_confidence must be in the range 0..100");
  }
  if (minimum_locked_beams_ < 3 || minimum_locked_beams_ > 4) {
    throw std::invalid_argument("minimum_locked_beams must be in the range 3..4");
  }
}

Dvl75Driver::AxisSigns Dvl75Driver::parse_axis_sign(
  const std::vector<double> & signs, const char * parameter_name)
{
  if (signs.size() != 3U || std::any_of(signs.begin(), signs.end(), [](double value) {
        return !std::isfinite(value) || value == 0.0;
      })) {
    throw std::invalid_argument(
      std::string(parameter_name) + " must contain three finite non-zero values");
  }
  return {signs[0], signs[1], signs[2]};
}

void Dvl75Driver::create_udp_socket()
{
  socket_ = std::make_unique<common::UdpSocket>();
  socket_->setBroadcast(true);
  socket_->setTimeout(kReceiveTimeout);
  if (!socket_->bind(bind_address_, listen_port_)) {
    socket_.reset();
    throw std::runtime_error("failed to bind DVL UDP socket");
  }
}

void Dvl75Driver::apply_configuration()
{
  std::vector<std::string> commands;
  if (!device_host_address_.empty()) {
    commands.push_back("HOST-ADDRESS " + device_host_address_);
  }
  commands.emplace_back("SEND-DVEXT ON");
  commands.emplace_back("SEND-DVPDL ON");
  commands.insert(commands.end(), startup_commands_.begin(), startup_commands_.end());
  for (const auto & command : commands) {
    send_command(command);
  }
  RCLCPP_WARN(
    get_logger(), "Applied %zu persistent DVL configuration command(s).", commands.size());
}

void Dvl75Driver::send_command(const std::string & command)
{
  const std::string normalized = trim(command);
  if (normalized.empty()) {
    throw std::invalid_argument("command must not be empty");
  }
  if (normalized.find_first_of("\r\n") != std::string::npos) {
    throw std::invalid_argument("command must be a single line");
  }
  if (disables_required_output(normalized)) {
    throw std::invalid_argument("DVEXT and DVPDL output are required and cannot be disabled");
  }

  const std::string payload = normalized + "\n";
  const std::vector<std::uint8_t> bytes(payload.begin(), payload.end());
  if (!socket_) {
    throw std::runtime_error("UDP socket is closed");
  }
  const ssize_t sent_size = socket_->sendTo(bytes, dvl_address_, dvl_command_port_);
  if (sent_size < 0) {
    throw std::runtime_error("sendto failed");
  }
  if (static_cast<std::size_t>(sent_size) != bytes.size()) {
    throw std::runtime_error("sendto sent an incomplete UDP datagram");
  }
  if (normalized == "SEND-DVPDL ON") {
    std::lock_guard<std::mutex> lock(data_mutex_);
    dvpdl_enable_requested_at_ = std::chrono::steady_clock::now();
  }
  RCLCPP_INFO(get_logger(), "Sent DVL command: %s", normalized.c_str());
}

void Dvl75Driver::receive_loop()
{
  try {
    while (!stop_requested_.load()) {
      const auto datagram = socket_->receiveFrom(kPacketBufferSize);
      if (!datagram) {
        continue;
      }
      const auto received_at = std::chrono::steady_clock::now();
      const auto received_stamp = static_cast<builtin_interfaces::msg::Time>(get_clock()->now());
      if (dvl_address_ != datagram->sender_address) {
        RCLCPP_DEBUG(
          get_logger(), "Ignoring packet from %s; expected DVL at %s",
          datagram->sender_address.c_str(), dvl_address_.c_str());
        continue;
      }
      process_packet(
        std::string(datagram->data.begin(), datagram->data.end()), received_at, received_stamp);
    }
  } catch (const std::exception & error) {
    if (!stop_requested_.load()) {
      RCLCPP_ERROR(get_logger(), "DVL UDP receive failed: %s", error.what());
    }
  }
}

void Dvl75Driver::process_packet(
  const std::string & packet, const SteadyTime & received_at,
  const builtin_interfaces::msg::Time & received_stamp)
{
  std::stringstream sentences(packet);
  std::string sentence;
  while (std::getline(sentences, sentence)) {
    if (trim(sentence).empty()) {
      continue;
    }
    Measurement measurement;
    try {
      measurement = parse_sentence(sentence, validate_checksum_);
    } catch (const ParseError & error) {
      RCLCPP_WARN(get_logger(), "Discarded invalid DVL packet: %s", error.what());
      publish_error_dvl(received_stamp);
      continue;
    }

    if (const auto * dvext = std::get_if<Dvext>(&measurement)) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_dvext_ = *dvext;
      last_dvext_time_ = received_at;
      continue;
    }
    const auto * dvpdl = std::get_if<Dvpdl>(&measurement);
    if (dvpdl == nullptr) {
      continue;
    }

    std::optional<Dvext> dvext;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_dvpdl_time_ = received_at;
      timeout_reported_ = false;
      dvpdl_enable_requested_at_.reset();
      if (received_at - last_dvext_time_ <= std::chrono::duration<double>(dvext_timeout_s_)) {
        dvext = last_dvext_;
      }
    }
    const auto position_delta = apply_axis_sign(dvpdl->position_delta, position_axis_sign_);
    const auto angle_delta = apply_axis_sign(dvpdl->angle_delta, angle_axis_sign_);
    publish_dvl75(dvext, *dvpdl, position_delta, angle_delta, received_stamp);
    publish_dvpdl(*dvpdl, dvext, position_delta, received_stamp);
  }
}

void Dvl75Driver::publish_dvpdl(
  const Dvpdl & dvpdl, const std::optional<Dvext> & dvext,
  const std::array<double, 3> & position_delta,
  const builtin_interfaces::msg::Time & received_stamp)
{
  driver_msgs::msg::DVL message;
  set_message_header(message, received_stamp, frame_id_);
  const std::size_t locked_beam_count =
    dvext
      ? static_cast<std::size_t>(std::count(dvext->beam_lock.begin(), dvext->beam_lock.end(), true))
      : 0U;
  const bool measurement_valid =
    dvext.has_value() && dvext->bottom_lock &&
    locked_beam_count >= static_cast<std::size_t>(minimum_locked_beams_) &&
    dvpdl.confidence >= static_cast<std::uint8_t>(minimum_confidence_) && dvpdl.delta_time_us > 0U;
  message.velocity_valid = measurement_valid;
  if (measurement_valid) {
    const double delta_seconds = static_cast<double>(dvpdl.delta_time_us) / 1'000'000.0;
    set_vector(
      message.velocity, {position_delta[0] / delta_seconds, position_delta[1] / delta_seconds,
                         position_delta[2] / delta_seconds});
    message.altitude = dvext->altitude;
    message.status.id = locked_beam_count == 4U ? common_msgs::msg::Status::NORMAL
                                                : common_msgs::msg::Status::WARNING;
  } else {
    message.status.id = common_msgs::msg::Status::ERROR;
  }
  report_tracking_status(message.status.id, locked_beam_count, dvpdl.confidence);
  dvl_publisher_->publish(message);
}

void Dvl75Driver::publish_dvl75(
  const std::optional<Dvext> & dvext, const Dvpdl & dvpdl,
  const std::array<double, 3> & position_delta, const std::array<double, 3> & angle_delta,
  const builtin_interfaces::msg::Time & received_stamp)
{
  blue_rov_msgs::msg::DVL75 message;
  set_message_header(message, received_stamp, frame_id_);
  if (dvext) {
    message.dvext_valid = true;
    message.bottom_lock = dvext->bottom_lock;
    message.data_skips = dvext->data_skips;
    message.velocity_up = dvext->velocity_up;
    message.altitude = dvext->altitude;
    message.velocity_north = dvext->velocity_north;
    message.velocity_east = dvext->velocity_east;
    message.beam_lock = dvext->beam_lock;
    message.beam_velocity = dvext->beam_velocity;
    message.beam_range = dvext->beam_range;
  }
  message.dvpdl_valid = true;
  message.device_time_us = dvpdl.device_time_us;
  message.delta_time_us = dvpdl.delta_time_us;
  set_vector(message.angle_delta, angle_delta);
  set_vector(message.position_delta, position_delta);
  message.confidence = dvpdl.confidence;
  dvl75_publisher_->publish(message);
}

void Dvl75Driver::publish_error_dvl(const builtin_interfaces::msg::Time & stamp)
{
  driver_msgs::msg::DVL message;
  set_message_header(message, stamp, frame_id_);
  message.velocity_valid = false;
  message.status.id = common_msgs::msg::Status::ERROR;
  dvl_publisher_->publish(message);
}

void Dvl75Driver::timeout_callback()
{
  const auto now = std::chrono::steady_clock::now();
  std::optional<SteadyTime> enabled_at;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (now - last_dvpdl_time_ <= std::chrono::duration<double>(timeout_s_) || timeout_reported_) {
      return;
    }
    timeout_reported_ = true;
    enabled_at = dvpdl_enable_requested_at_;
  }
  publish_error_dvl(static_cast<builtin_interfaces::msg::Time>(get_clock()->now()));
  if (enabled_at) {
    const std::chrono::duration<double> elapsed = now - *enabled_at;
    RCLCPP_ERROR(
      get_logger(),
      "DVL-75 configuration error: no DVPDL received after SEND-DVPDL ON (%.2f s, target=%s:%d). "
      "UDP send success does not confirm DVL receipt.",
      elapsed.count(), dvl_address_.c_str(), dvl_command_port_);
  } else {
    RCLCPP_ERROR(
      get_logger(), "DVL-75 timeout: no DVPDL sentence received within %.2f s", timeout_s_);
  }
}

void Dvl75Driver::report_tracking_status(
  std::uint8_t status, std::size_t locked_beam_count, std::uint8_t confidence)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (last_tracking_status_ && *last_tracking_status_ == status) {
      return;
    }
    last_tracking_status_ = status;
  }
  if (status == common_msgs::msg::Status::ERROR) {
    RCLCPP_ERROR(
      get_logger(), "DVL-75 tracking status=%u, locked_beams=%zu, confidence=%u",
      static_cast<unsigned int>(status), locked_beam_count, static_cast<unsigned int>(confidence));
  } else if (status == common_msgs::msg::Status::WARNING) {
    RCLCPP_WARN(
      get_logger(), "DVL-75 tracking status=%u, locked_beams=%zu, confidence=%u",
      static_cast<unsigned int>(status), locked_beam_count, static_cast<unsigned int>(confidence));
  } else {
    RCLCPP_INFO(
      get_logger(), "DVL-75 tracking status=%u, locked_beams=%zu, confidence=%u",
      static_cast<unsigned int>(status), locked_beam_count, static_cast<unsigned int>(confidence));
  }
}

Dvl75Driver::AxisSigns Dvl75Driver::apply_axis_sign(
  const std::array<double, 3> & values, const AxisSigns & signs)
{
  return {values[0] * signs[0], values[1] * signs[1], values[2] * signs[2]};
}

bool Dvl75Driver::disables_required_output(const std::string & command)
{
  std::stringstream stream(command);
  std::string name;
  std::string value;
  std::string extra;
  stream >> name >> value >> extra;
  const auto to_upper = [](unsigned char character) {
    return static_cast<char>(std::toupper(character));
  };
  std::transform(name.begin(), name.end(), name.begin(), to_upper);
  std::transform(value.begin(), value.end(), value.begin(), to_upper);
  return extra.empty() && value == "OFF" && (name == "SEND-DVEXT" || name == "SEND-DVPDL");
}

}  // namespace dvl75_driver

RCLCPP_COMPONENTS_REGISTER_NODE(dvl75_driver::Dvl75Driver)
