/**
 * @file timeout.hpp
 * @brief timeout library
 * @author R.Ohnishi
 * @date 2025/05/11
 *
 * @details タイムアウト処理用のライブラリ
 **************************************************/

#ifndef _TIMEOUT_HPP
#define _TIMEOUT_HPP

#include <rclcpp/rclcpp.hpp>

/**
 * @namespace timer
 * @brief For timer
 */
namespace timer
{
/**
 * @brief timeout class
 */
class Timeout
{
private:
  rclcpp::Time start_time;
  rclcpp::Duration elapsed_time;
  int64_t timeout;

public:
  /**
   * @brief timeout settings
   * @param start_time measurement start time
   * @param timeout time until timeout [ns]
   * @details Settings such as start time and timeout.
   */
  explicit Timeout(rclcpp::Time start_time, int64_t timeout);

  /**
   * @brief reset start time
   * @param start_time measurement start time
   * @return non
   * @details Update the start time and reset the timeout measurement.
   */
  void reset(rclcpp::Time start_time);

  /**
   * @brief Determine if it has time out.
   * @param now current time
   * @return bool true if timeout, otherwise false
   */
  bool is_timeout(rclcpp::Time now);

  /**
   * @brief Get elapsed time
   * @return int64_t elapsed time [ns]
   */
  int64_t get_elapsed_time();

  /**
   * @brief Get timeout
   * @return int64_t timeout [ns]
   */
  int64_t get_timeout();
};

}  // namespace timer

#endif
