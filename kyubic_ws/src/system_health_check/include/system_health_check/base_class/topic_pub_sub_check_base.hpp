#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <system_health_check/base_class/system_health_check_base.hpp>
#include <thread>

namespace system_health_check
{

// 比較モードの定義
enum class ComparisonMode
{
  GREATER_OR_EQUAL,  // count >= expected (デフォルト: 最低でもこれだけいる)
  EQUAL              // count == expected (厳密にこれだけいる)
};

/**
 * @brief Abstract base class for checking Topic Publisher/Subscriber counts.
 */
class TopicCountCheckBase : public system_health_check::SystemCheckBase
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
      // 派生クラスで実装されたカウント関数を呼ぶ
      size_t current_count = get_count(node, topic_name_);

      // 比較ロジック
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

      // タイムアウト判定
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

  std::string report([[maybe_unused]] rclcpp::Node::SharedPtr node) override
  {
    return "Topic " + get_target_type() + ": " + topic_name_ + " | " + status_msg_;
  }

  /**
   * @brief Configure the check settings.
   * @param mode Comparison mode (GREATER_OR_EQUAL or EQUAL).
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

protected:
  // 派生クラスで実装: 具体的なカウント方法
  virtual size_t get_count(rclcpp::Node::SharedPtr node, const std::string & topic) = 0;
  virtual std::string get_target_type() const = 0;

  std::string topic_name_;
  uint32_t timeout_ms_ = 1000;
  size_t expected_count_ = 1;
  ComparisonMode mode_ = ComparisonMode::GREATER_OR_EQUAL;
  std::string status_msg_;
};

/**
 * @brief Checks if there are any PUBLISHERS for the specified topic.
 */
class TopicPublisherCheckBase : public TopicCountCheckBase
{
protected:
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
protected:
  size_t get_count(rclcpp::Node::SharedPtr node, const std::string & topic) override
  {
    return node->count_subscribers(topic);
  }
  std::string get_target_type() const override { return "Subscriber"; }
};

}  // namespace system_health_check
