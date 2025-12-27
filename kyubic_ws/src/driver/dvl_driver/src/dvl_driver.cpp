/**
 * @file dvl_driver.cpp
 * @brief DVL driver Node Definition
 * @author R.Ohnishi
 * @date 2024/10/30
 *
 * @details DVL(Path Finder) のPD0/PD5データを取得して，Topicを流す
 *****************************************************************/

#include "dvl_driver/dvl_driver.hpp"

using namespace std::chrono_literals;

namespace dvl_driver
{

namespace
{
constexpr int16_t INVALID_VELOCITY = -32768;
constexpr double SCALE_VELOCITY = 0.001;  // mm/s -> m/s
constexpr double SCALE_RANGE_CM = 0.01;   // cm -> m
}  // namespace

DVLDriver::DVLDriver() : Node("dvl_driver")
{
  // Get parameter from server
  address = this->declare_parameter("ip_address", "192.168.1.100");
  listener_port = this->declare_parameter("listener_port", 1034);
  sender_port = this->declare_parameter("sender_port", 1033);
  timeout_ms = this->declare_parameter("timeout_ms", 0);
  turning = this->declare_parameter("turning", false);

  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ms * 1e6);

  // Connect TCP
  listener_ = std::make_shared<path_finder::Listener>(address.c_str(), listener_port, 500);
  sender_ = std::make_shared<path_finder::Sender>(address.c_str(), sender_port, 500);
  RCLCPP_INFO(this->get_logger(), "DVL connection initialized to %s", address.c_str());

  // Setup DVL (Wakeup)
  if (!setup()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to setup DVL. Exiting.");
    exit(1);
  }

  // Create publisher
  pub_ = create_publisher<driver_msgs::msg::DVL>("dvl", rclcpp::SensorDataQoS());

  // Timer for main loop (5Hz)
  timer_ = create_wall_timer(200ms, std::bind(&DVLDriver::update, this));

  // Service for raw commands
  srv_ = create_service<driver_msgs::srv::Command>(
    "~/command",
    std::bind(&DVLDriver::sendCommandCallback, this, std::placeholders::_1, std::placeholders::_2));
}

DVLDriver::~DVLDriver()
{
  if (sender_->send_break_cmd()) {
    RCLCPP_INFO(this->get_logger(), "Sent break command for shutdown.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to send the break command during shutdown.");
  }
}

bool DVLDriver::setup()
{
  // Send BREAK to wake up the DVL
  if (!sender_->send_break_cmd()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send the break command.");
    return false;
  }

  // Read wakeup banner
  unsigned char buffer[1024] = {'\0'};
  sender_->read(buffer, sizeof(buffer));

  RCLCPP_INFO(this->get_logger(), "DVL Wakeup Response: %s", buffer);
  return true;
}

bool DVLDriver::_update()
{
  // If in manual ping mode (CFx0xxx), we need to trigger a ping.
  if (!sender_->send_ping_cmd()) {
    RCLCPP_ERROR(this->get_logger(), "ping faild");
    return false;
  }

  if (!listener_->listen()) {
    // Listen failed or no data available yet
    return false;
  }

  return true;
}

void DVLDriver::update()
{
  if (command_mode) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Command Mode Active - Skipping data update");
    return;
  }

  auto msg = std::make_unique<driver_msgs::msg::DVL>();
  bool data_received = false;

  if (DVLDriver::_update()) {
    // PD0 Data Handling
    if (listener_->has_pd0_data()) {
      _extractPd0Data(listener_->get_pd0_data(), *msg);
      data_received = true;
    }
    // PD5 Data Handling
    else if (listener_->has_pd5_data()) {
      _extractPd5Data(listener_->get_pd5_data(), *msg);
      data_received = true;
    }
  }

  if (data_received) {
    timeout_->reset(this->get_clock()->now());

    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = "pathfinder";

    // Determine Status
    std::string qual_str = "4-Beam";

    // Error conditions: Invalid XYZ velocity OR Leak Detected
    if (!msg->velocity_valid || msg->leak_a || msg->leak_b) {
      msg->status.id = common_msgs::msg::Status::ERROR;
      qual_str = "++ERROR++";
    }
    // Warning condition: Valid XYZ but Invalid Error Velocity (3-Beam solution)
    else if (msg->velocity_error == INVALID_VELOCITY) {
      msg->status.id = common_msgs::msg::Status::WARNING;
      qual_str = "3-Beam";
    } else {
      msg->status.id = common_msgs::msg::Status::NORMAL;
    }

    RCLCPP_INFO(
      this->get_logger(), "[%s] Vel: [%5.2f, %5.2f, %5.2f]  Alt: %5.2f  Leak_A: %d  Leak_B: %d",
      qual_str.c_str(), msg->velocity.x, msg->velocity.y, msg->velocity.z, msg->altitude,
      msg->leak_a, msg->leak_b);

    pub_->publish(std::move(msg));

  } else {
    if (timeout_->is_timeout(this->get_clock()->now())) {
      msg->header.stamp = this->get_clock()->now();
      msg->status.id = common_msgs::msg::Status::ERROR;
      pub_->publish(std::move(msg));

      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "DVL Timeout - No data received.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed DVL data acquisition.");
    }
  }
}

void DVLDriver::_extractPd0Data(
  const std::shared_ptr<path_finder::pd0::Pd0Ensemble> & data, driver_msgs::msg::DVL & msg)
{
  // System Config & Leak Status
  msg.system_config = static_cast<uint8_t>(data->fixed_leader.system_config & 0xFF);
  msg.leak_a = (data->variable_leader.leak_status & 0x01) != 0;
  msg.leak_b = (data->variable_leader.leak_status & 0x04) != 0;

  // Velocity
  _convertVelocity(data->bottom_track.velocity, msg);

  // Range & Status
  float ranges[4];
  for (int i = 0; i < 4; ++i) {
    ranges[i] = data->bottom_track.range[i] * SCALE_RANGE_CM;

    // Threshold based status
    msg.btm_status_echo_amplitude[i] = (data->bottom_track.eval_amp[i] > 20);
    msg.btm_status_correlation[i] = (data->bottom_track.correlation[i] > 64);
  }
  msg.altitude = _calculate_average_altitude(ranges);

  // Debug Logging for PD0
  if (turning) {
    const auto & btm = data->bottom_track;
    RCLCPP_INFO(
      this->get_logger(),
      "RSSI: %3u, %3u, %3u, %3u  Corr: %3u, %3u, %3u, %3u  Range: %3u, %3u, %3u, %3u  Vel: %6d, "
      "%6d, %6d, %6d Error: %6d",
      btm.rssi_amp[0], btm.rssi_amp[1], btm.rssi_amp[2], btm.rssi_amp[3], btm.correlation[0],
      btm.correlation[1], btm.correlation[2], btm.correlation[3], btm.range[0], btm.range[1],
      btm.range[2], btm.range[3], btm.velocity[0], btm.velocity[1], btm.velocity[2],
      btm.velocity[3], msg.velocity_error);
  }
}

void DVLDriver::_extractPd5Data(
  const std::shared_ptr<path_finder::pd5::Pd5Ensemble> & data, driver_msgs::msg::DVL & msg)
{
  // System Config & Leak Status
  msg.system_config = data->system_config;
  msg.leak_a = (data->bit_results & 0x01) != 0;
  msg.leak_b = (data->bit_results & 0x04) != 0;

  // Velocity
  _convertVelocity(data->btm_velocity, msg);

  // Range & Status
  float ranges[4];
  for (int i = 0; i < 4; ++i) {
    ranges[i] = data->btm_range[i] * SCALE_RANGE_CM;

    // Bitwise status extraction
    msg.btm_status_correlation[i] = (data->bottom_status >> i) & 0x01;
    msg.btm_status_echo_amplitude[i] = (data->bottom_status >> (i * 2 + 1)) & 0x01;
  }
  msg.altitude = _calculate_average_altitude(ranges);
}

template <typename T>
void DVLDriver::_convertVelocity(const T & raw_vel, driver_msgs::msg::DVL & msg)
{
  int16_t vx = raw_vel[0];
  int16_t vy = raw_vel[1];
  int16_t vz = raw_vel[2];

  msg.velocity.x = vx * SCALE_VELOCITY;
  msg.velocity.y = vy * SCALE_VELOCITY;
  msg.velocity.z = vz * SCALE_VELOCITY;
  msg.velocity_error = raw_vel[3];

  // Validity Check (3-Beam solution check)
  if (vx != INVALID_VELOCITY && vy != INVALID_VELOCITY && vz != INVALID_VELOCITY) {
    msg.velocity_valid = true;
  } else {
    msg.velocity_valid = false;
  }
}

double DVLDriver::_calculate_average_altitude(const float ranges[4])
{
  double sum = 0.0;
  int count = 0;
  for (int i = 0; i < 4; ++i) {
    if (ranges[i] > 0.001) {  // Valid range check (> 1mm)
      sum += ranges[i];
      count++;
    }
  }
  return (count > 0) ? (sum / count) : 0.0;
}

void DVLDriver::sendCommandCallback(
  const driver_msgs::srv::Command::Request::SharedPtr request,
  driver_msgs::srv::Command::Response::SharedPtr response)
{
  command_mode = true;
  RCLCPP_INFO(this->get_logger(), "Service request command: '%s'", request->command.c_str());

  // Switch command mode
  if (request->command == "===") break_cmd = true;

  if (request->command == "CS") {
    break_cmd = false;
    command_mode = false;
    response->output = "Start ping";
    return;
  }

  sender_->flush_buffer();
  if (!request->command.empty()) {
    // Send command with wait time
    if (sender_->send_cmd(request->command, 1)) {
      unsigned char buffer[4096] = {'\0'};
      size_t len = sender_->read(buffer, sizeof(buffer));

      if (len > 0) {
        std::string data(reinterpret_cast<const char *>(buffer), len);

        // Simple response cleaning
        // Remove echoed command if present
        size_t pos = data.find(request->command);
        if (pos != std::string::npos) {
          data = data.substr(pos + request->command.size() + 2);
        }
        // Remove prompt
        if ((pos = data.find(">")) != std::string::npos) {
          data.erase(data.begin() + pos, data.end());
        }

        response->output = data;
      } else {
        response->output = "No response from DVL";
      }
    } else {
      response->output = "Failed to send command: " + request->command;
    }
  }

  if (!break_cmd) command_mode = false;
}

}  // namespace dvl_driver

/**
 * @brief Main entry point
 */
int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<dvl_driver::DVLDriver>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Exception in DVL Driver: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
