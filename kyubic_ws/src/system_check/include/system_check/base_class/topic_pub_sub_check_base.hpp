#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <system_check/base_class/system_check_base.hpp>
#include <thread>

namespace system_check
{

/**
 * @brief Checks if there are any publishers for the specified topic.
 */
class TopicPublisherCheckBase : public system_check::SystemCheckBase
{
public:
  virtual bool check(rclcpp::Node::SharedPtr node) override
  {
    if (topic_name_.empty()) {
      status_msg_ = "Topic name not set. Please invoke `set_config()` function.";
      return false;
    }

    auto start_time = std::chrono::steady_clock::now();
    auto timeout_duration = std::chrono::milliseconds(timeout_ms_);

    while (true) {
      // Get the number of publishers for the topic
      size_t pub_count = node->count_publishers(topic_name_);

      if (pub_count >= expected_count_) {
        status_msg_ = "Found " + std::to_string(pub_count) + " publisher(s).";
        return true;  // PASS
      }

      // Check timeout
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (elapsed > timeout_duration) {
        status_msg_ = "Timeout. Found " + std::to_string(pub_count) +
                      " publisher(s) (Expected >= " + std::to_string(expected_count_) + ").";
        return false;  // FAIL
      }

      // Sleep briefly
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  std::string report([[maybe_unused]] rclcpp::Node::SharedPtr node) override
  {
    return "Topic: " + topic_name_ + " | " + status_msg_;
  }

  void set_config(
    const std::string topic_name, const uint32_t timeout_ms, const size_t expected_count = 1)
  {
    topic_name_ = topic_name;
    timeout_ms_ = timeout_ms;
    expected_count_ = expected_count;
  };

private:
  std::string topic_name_;
  uint32_t timeout_ms_ = 1000;
  size_t expected_count_ = 1;
  std::string status_msg_;
};

/**
 * @brief Checks if there are any subscribers for the specified topic.
 * Note: This does not require a template because it checks the ROS graph, not the message content.
 */
class TopicSubscriberCheckBase : public system_check::SystemCheckBase
{
public:
  virtual bool check(rclcpp::Node::SharedPtr node) override
  {
    if (topic_name_.empty()) {
      status_msg_ = "Topic name not set. Please invoke `set_config()` function.";
      return false;
    }

    auto start_time = std::chrono::steady_clock::now();
    auto timeout_duration = std::chrono::milliseconds(timeout_ms_);

    while (true) {
      // Get the number of subscribers for the topic
      size_t sub_count = node->count_subscribers(topic_name_);

      if (sub_count >= expected_count_) {
        status_msg_ = "Found " + std::to_string(sub_count) + " subscriber(s).";
        return true;  // PASS
      }

      // Check timeout
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (elapsed > timeout_duration) {
        status_msg_ = "Timeout. Found " + std::to_string(sub_count) +
                      " subscriber(s) (Expected >= " + std::to_string(expected_count_) + ").";
        return false;  // FAIL
      }

      // Sleep briefly to avoid busy loop
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  std::string report([[maybe_unused]] rclcpp::Node::SharedPtr node) override
  {
    return "Topic: " + topic_name_ + " | " + status_msg_;
  }

  void set_config(
    const std::string topic_name, const uint32_t timeout_ms, const size_t expected_count = 1)
  {
    topic_name_ = topic_name;
    timeout_ms_ = timeout_ms;
    expected_count_ = expected_count;
  };

private:
  std::string topic_name_;
  uint32_t timeout_ms_ = 1000;
  size_t expected_count_ = 1;
  std::string status_msg_;
};

}  // namespace system_check
