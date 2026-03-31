/**
 * @file localization_error.cpp
 * @brief Ring a bell when localization error is detected
 * @author R.Ohnishi
 * @date 2026/01/31
 *
 * @details localizationのオドメトリが不正な場合にベルを鳴らす
 ***************************************************************/

#ifndef _LOCALIZATION_ERROR_HPP
#define _LOCALIZATION_ERROR_HPP

#include <alsa/asoundlib.h>

#include <rclcpp/rclcpp.hpp>

#include "localization_msgs/msg/odometry.hpp"

namespace tools::system_alert
{

/**
 * @brief Component node to monitor localization status and play alert sound on error.
 */
class LocalizationError : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Localization Error object.
   *
   * @param options Node options for component loading.
   */
  explicit LocalizationError(const rclcpp::NodeOptions & options);

  /**
   * @brief Destroy the Localization Error object and release audio resources.
   */
  ~LocalizationError();

private:
  /**
   * @brief Initialize ALSA PCM device parameters.
   */
  void init_audio_device();

  /**
   * @brief Callback function for Odometry topic.
   * Checks if any sensor status reports an ERROR.
   *
   * @param msg The received Odometry message.
   */
  void odometry_callback(const localization_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Generate and play a warning tone (Sine wave).
   *
   * @param freq Frequency of the tone (Hz).
   * @param duration Duration of the tone (seconds).
   */
  void play_alert_sound(double freq, double duration);

  // Member variables
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_odom_;
  snd_pcm_t * pcm_handle_{nullptr};
  const char * device_name_{"default"};

  rclcpp::Time last_alert_time_;
};

}  // namespace tools::system_alert

#endif  // _LOCALIZATION_ERROR_HPP
