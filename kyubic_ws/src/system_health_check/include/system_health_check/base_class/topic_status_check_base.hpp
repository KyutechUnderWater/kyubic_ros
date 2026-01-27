/**
 * @file topic_status_check_base.hpp
 * @brief Template class for checking topic status
 * @author R.Ohnishi
 * @date 2026/01/21
 *
 * @details Topicの状態をチェックする
 **********************************************/

#ifndef _TOPIC_STATUS_CHECK_BASE_HPP
#define _TOPIC_STATUS_CHECK_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <string>
#include <system_health_check/base_class/system_check_base.hpp>

namespace system_health_check::base
{

/**
  * @brief Template class for checking topic reception.
  * @tparam MessageT The message type to check (e.g., std_msgs::msg::Bool).
  */
template <typename MessageT>
class TopicStatusCheckBase : public SystemCheckBase
{
protected:
  /**
   * @brief Set the status id for cache check
   * @param id status id (e.g., "depth_status")
   */
  void set_status_id(const std::string id) { status_id_ = id; }

  void set_config(const std::string topic_name, const uint32_t timeout_ms)
  {
    topic_name_ = topic_name;
    timeout_ms_ = timeout_ms;
  };

  virtual void prepare_check(rclcpp::Node::SharedPtr node) override = 0;

  /**
   * @brief Virtual function to verify message content.
   * Defaults to "OK if received".
   * Override in a derived class to inspect content (e.g., msg->data == true).
   */
  virtual bool validate([[maybe_unused]] const MessageT & msg) { return true; }

private:
  std::string topic_name_;
  uint32_t timeout_ms_ = 0;
  std::string status_msg_;

  std::string status_id_ = "";
  std::string get_unique_id() const override
  {
    if (status_id_.empty()) {
      return "";
    }
    return "TopicStatusCheckBase::" + topic_name_ + "::" + status_id_;
  }

  bool check_impl(rclcpp::Node::SharedPtr node) override
  {
    if (topic_name_.empty() || timeout_ms_ == 0) {
      status_msg_ = "Not set topic_name or timeout. Please invoke `set_config()` function.";
      return false;  // FAIL (no received)
    }

    MessageT msg;

    // Wait message received
    bool received = rclcpp::wait_for_message<MessageT>(
      msg, node, topic_name_,
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timeout_ms_ * 1e-3)));

    if (received) {
      if (validate(msg)) {
        status_msg_ = "Received & Validated";
        return true;  // PASS
      } else {
        status_msg_ = "Received but invalid content";
        return false;  // FAIL (not validate)
      }
    } else {
      status_msg_ = "Timeout";
      return false;  // FAIL (no received)
    }
  }

  std::string report_impl([[maybe_unused]] rclcpp::Node::SharedPtr node) override
  {
    return "Topic: " + topic_name_ + " | " + status_msg_;
  }
};

}  // namespace system_health_check::base

#endif
