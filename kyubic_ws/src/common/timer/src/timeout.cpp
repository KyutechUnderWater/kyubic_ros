/**
 * @file timeout.cpp
 * @brief timeout library
 * @author R.Ohnishi
 * @date 2025/05/11
 *
 * @details タイムアウト処理用のライブラリ
 **************************************************/

#include "timer/timeout.hpp"

namespace timer
{

Timeout::Timeout(rclcpp::Time start_time, int64_t timeout)
: start_time(start_time), elapsed_time(0, 0), timeout(timeout)
{
}

void Timeout::reset(rclcpp::Time start_time) { this->start_time = start_time; }

bool Timeout::is_timeout(rclcpp::Time now)
{
  elapsed_time = now - start_time;
  if (timeout != 0 && elapsed_time.nanoseconds() > timeout) {
    return true;
  }
  return false;
}

int64_t Timeout::get_elapsed_time() { return elapsed_time.nanoseconds(); }
int64_t Timeout::get_timeout() { return timeout; }

}  // namespace timer
