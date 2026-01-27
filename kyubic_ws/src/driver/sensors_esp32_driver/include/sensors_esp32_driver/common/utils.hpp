/**
 * @file utils.hpp
 * @brief Implement utils for sensors_esp32_driver
 * @author R.Ohnishi
 * @date 2025/11/22
 *
 * @details センサ基板との通信ドライバで使うユーティリティ
 ********************************************************/

#ifndef _UTILS_HPP
#define _UTILS_HPP

#include <common_msgs/msg/status.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <timer/timeout.hpp>

namespace driver::sensors_esp32_driver
{

/**
 * @brief Template function for checking timeouts.
 * @tparam MsgT The type of the message to be published (e.g., driver_msgs::msg::Depth).
 * @param node Pointer to the node, used for logging and retrieving the current time.
 * @param mutex Mutex for thread safety (mutual exclusion).
 * @param timeout The timeout management object.
 * @param pub The publisher instance.
 * @param sensor_name The name of the sensor used in log messages (e.g., "Depth").
 */
template <typename MsgT>
void check_timeout(
  rclcpp::Node * node, std::mutex & mutex, std::shared_ptr<timer::Timeout> & timeout,
  typename rclcpp::Publisher<MsgT>::SharedPtr & pub, const std::string & sensor_name)
{
  bool is_timeout;
  int64_t timeout_ns;
  int64_t elapsed_time;
  {
    std::lock_guard<std::mutex> lock(mutex);
    is_timeout = timeout->is_timeout(node->get_clock()->now());
    timeout_ns = timeout->get_timeout();
    elapsed_time = timeout->get_elapsed_time();
  }

  if (is_timeout) {
    auto msg = std::make_unique<MsgT>();
    msg->header.stamp = node->get_clock()->now();
    msg->status.id = common_msgs::msg::Status::ERROR;

    pub->publish(std::move(msg));

    RCLCPP_ERROR_THROTTLE(
      node->get_logger(), *node->get_clock(), timeout_ns * 1e-6, "%s driver timeout: %lu [ms]",
      sensor_name.c_str(), (uint64_t)(timeout->get_elapsed_time() * 1e-6));
  } else if (elapsed_time > timeout_ns * 0.5) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), timeout_ns * 1e-6 / 2, "Failed to get %s data",
      sensor_name.c_str());
    return;
  }
}

}  // namespace driver::sensors_esp32_driver

#endif  // !_UTILS_HPP
