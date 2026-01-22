/**
 * @file system_check.hpp
 * @brief Check topic, service, action server, and other system
 * @author R.Ohnishi
 * @date 2026/01/21
 *
 * @details Topicの情報や各サーバが起動しているかなどのシステムをチェックする
 ***************************************************************************/

#ifndef _SYSTEM_CHECK_HPP
#define _SYSTEM_CHECK_HPP

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "system_check/base_class/system_check_base.hpp"

namespace system_check
{

class SystemCheck : public rclcpp::Node
{
public:
  explicit SystemCheck(const rclcpp::NodeOptions & options);

private:
  std::shared_ptr<pluginlib::ClassLoader<system_check::SystemCheckBase>> loader_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  void run_checks();
};

}  // namespace system_check

#endif  // !_SYSTEM_CHECK_HPP
