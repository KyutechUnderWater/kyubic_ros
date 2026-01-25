/**
 * @file topic_pub_sub_check_base.hpp
 * @brief Check publisher and subscriber counts
 * @author R.Ohnishi
 * @date 2026/01/21
 *
 * @details PublisherとSubscriberの数をチェックする
 *************************************************/

#ifndef _TOPIC_PUB_SUB_CHECK_BASE_HPP
#define _TOPIC_PUB_SUB_CHECK_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <system_health_check/base_class/system_check_base.hpp>
#include <thread>

namespace system_health_check::base
{

// Define comparison modes
enum class ComparisonMode
{
  GREATER_OR_EQUAL,  // count >= expected
  EQUAL              // count == expected
};

/**
 * @brief Abstract base class for checking Topic Publisher/Subscriber counts.
 */
class TopicCountCheckBase : public SystemCheckBase
{
protected:
  std::string topic_name_;
  uint32_t timeout_ms_ = 1000;
  size_t expected_count_ = 1;
  ComparisonMode mode_ = ComparisonMode::GREATER_OR_EQUAL;

  /**
   * @brief Configure the check settings.
   * @param topic_name Topic name to check.
   * @param timeout_ms Timeout in milliseconds.
   * @param expected_count Expected count of publishers/subscribers.
   * @param mode Comparison mode (GREATER_OR_EQUAL or EQUAL). Default is ComparisonMode::EQUAL.
   */
  void set_config(
    const std::string topic_name, const uint32_t timeout_ms, const size_t expected_count = 1,
    const ComparisonMode mode = ComparisonMode::EQUAL)
  {
    topic_name_ = topic_name;
    timeout_ms_ = timeout_ms;
    expected_count_ = expected_count;
    mode_ = mode;
  };

  virtual void prepare_check(rclcpp::Node::SharedPtr node) override = 0;
  virtual size_t get_count(rclcpp::Node::SharedPtr node, const std::string & topic) = 0;
  virtual std::string get_target_type() const = 0;

private:
  std::string status_msg_;

  bool check_impl(rclcpp::Node::SharedPtr node) override
  {
    if (topic_name_.empty()) {
      status_msg_ = "Topic name not set. Please invoke `set_config()` function.";
      return false;
    }

    auto start_time = std::chrono::steady_clock::now();
    auto timeout_duration = std::chrono::milliseconds(timeout_ms_);

    while (true) {
      // Inherited count method
      size_t current_count = get_count(node, topic_name_);

      // Comparison
      bool passed = false;
      if (mode_ == ComparisonMode::GREATER_OR_EQUAL) {
        passed = (current_count >= expected_count_);
      } else {
        passed = (current_count == expected_count_);
      }

      if (passed) {
        status_msg_ = "Found " + std::to_string(current_count) + " " + get_target_type() + "(s).";
        return true;  // PASS
      }

      // Check timeout
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (elapsed > timeout_duration) {
        std::string mode_str = (mode_ == ComparisonMode::GREATER_OR_EQUAL) ? ">=" : "==";
        status_msg_ = "Timeout. Found " + std::to_string(current_count) + " " + get_target_type() +
                      "(s) (Expected " + mode_str + " " + std::to_string(expected_count_) + ").";
        return false;  // FAIL
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  std::string report_impl([[maybe_unused]] rclcpp::Node::SharedPtr node) override
  {
    return "Topic " + get_target_type() + ": " + topic_name_ + " | " + status_msg_;
  }
};

/**
 * @brief Checks if there are any PUBLISHERS for the specified topic.
 */
class TopicPublisherCheckBase : public TopicCountCheckBase
{
private:
  std::string get_unique_id() const override
  {
    return "TopicPublisherCheckBase::" + topic_name_ + "::" + std::to_string(expected_count_) +
           "::" + std::to_string(static_cast<int>(mode_));
  }

  size_t get_count(rclcpp::Node::SharedPtr node, const std::string & topic) override
  {
    return node->count_publishers(topic);
  }

  std::string get_target_type() const override { return "Publisher"; }
};

/**
 * @brief Checks if there are any SUBSCRIBERS for the specified topic.
 */
class TopicSubscriberCheckBase : public TopicCountCheckBase
{
private:
  std::string get_unique_id() const override
  {
    return "TopicSubscriberCheckBase::" + topic_name_ + "::" + std::to_string(expected_count_) +
           "::" + std::to_string(static_cast<int>(mode_));
  }

  size_t get_count(rclcpp::Node::SharedPtr node, const std::string & topic) override
  {
    return node->count_subscribers(topic);
  }

  std::string get_target_type() const override { return "Subscriber"; }
};

}  // namespace system_health_check::base

#endif
