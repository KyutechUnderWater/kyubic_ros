#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <string>
#include <system_health_check/base_class/system_health_check_base.hpp>

namespace system_health_check
{

/**
  * @brief Template class for checking topic reception.
  * @tparam MessageT The message type to check (e.g., std_msgs::msg::Bool).
  */
template <typename MessageT>
class TopicStatusCheckBase : public system_health_check::SystemCheckBase
{
public:
  virtual bool check(rclcpp::Node::SharedPtr node) override
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

  std::string report([[maybe_unused]] rclcpp::Node::SharedPtr node) override
  {
    return "Topic: " + topic_name_ + " | " + status_msg_;
  }

  void set_config(const std::string topic_name, const uint32_t timeout_ms)
  {
    topic_name_ = topic_name;
    timeout_ms_ = timeout_ms;
  };

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
};

}  // namespace system_health_check
