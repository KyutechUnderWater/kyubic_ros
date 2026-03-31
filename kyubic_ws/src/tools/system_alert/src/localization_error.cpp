#include "system_alert/localization_error.hpp"

#include <cmath>
#include <vector>

namespace tools::system_alert
{

LocalizationError::LocalizationError(const rclcpp::NodeOptions & options)
: Node("localization_error", options)
{
  // Initialize audio device
  init_audio_device();

  // Initialize last alert time
  last_alert_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Create subscription
  sub_odom_ = this->create_subscription<localization_msgs::msg::Odometry>(
    "odom", 10, std::bind(&LocalizationError::odometry_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "LocalizationError node initialized.");
}

LocalizationError::~LocalizationError()
{
  if (pcm_handle_) {
    snd_pcm_drain(pcm_handle_);
    snd_pcm_close(pcm_handle_);
  }
}

void LocalizationError::init_audio_device()
{
  int err;
  // Open PCM device for playback
  if ((err = snd_pcm_open(&pcm_handle_, device_name_, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Cannot open audio device %s (%s)", device_name_, snd_strerror(err));
    return;
  }

  // Set parameters: 44100Hz, 16-bit Little Endian, 1 channel
  // Latency set to 100ms (100000us) for quicker response
  if (
    (err = snd_pcm_set_params(
       pcm_handle_, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED, 1, 44100, 1, 100000)) <
    0) {
    RCLCPP_ERROR(this->get_logger(), "Playback open error: %s", snd_strerror(err));
  }
}

void LocalizationError::odometry_callback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  // Constants for status check (using the enum from message definition)
  const uint8_t STATUS_ERROR = common_msgs::msg::Status::ERROR;

  bool is_depth_error = (msg->status.depth.id == STATUS_ERROR);
  bool is_imu_error = (msg->status.imu.id == STATUS_ERROR);
  bool is_dvl_error = (msg->status.dvl.id == STATUS_ERROR);

  if (is_depth_error || is_imu_error || is_dvl_error) {
    auto now = this->now();

    if ((now - last_alert_time_).seconds() > 2.0) {
      RCLCPP_WARN(this->get_logger(), "Localization Error Detected! Playing alert sound.");

      play_alert_sound(880.0, 0.2);
      play_alert_sound(880.0, 0.2);

      last_alert_time_ = now;

    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "Localization Error persisting...");
    }
  }
}

void LocalizationError::play_alert_sound(double freq, double duration)
{
  if (!pcm_handle_) return;

  // Simple sine wave generation
  int rate = 44100;
  int frames = static_cast<int>(rate * duration);
  std::vector<short> buffer(frames);

  for (int i = 0; i < frames; i++) {
    double t = static_cast<double>(i) / rate;
    // Amplitude 30000 (near max for 16-bit signed)
    buffer[i] = static_cast<short>(30000.0 * sin(2.0 * M_PI * freq * t));
  }

  // Recover from potential underrun/suspension state before writing
  int err = snd_pcm_prepare(pcm_handle_);
  if (err < 0) {
    RCLCPP_ERROR(this->get_logger(), "Prepare error: %s", snd_strerror(err));
    return;
  }

  // Write audio data
  snd_pcm_sframes_t frames_written = snd_pcm_writei(pcm_handle_, buffer.data(), frames);

  if (frames_written < 0) {
    snd_pcm_recover(pcm_handle_, frames_written, 1);
  } else {
    snd_pcm_drain(pcm_handle_);
  }
}

}  // namespace tools::system_alert

// Register the component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tools::system_alert::LocalizationError)
