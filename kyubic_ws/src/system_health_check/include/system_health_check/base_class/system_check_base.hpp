/**
 * @file system_check_base.hpp
 * @brief Abstruct Class of check process
 * @author R.Ohnishi
 * @date 2026/01/21
 *
 * @details 各種チェック処理のベースとなるクラス
 **********************************************/

#ifndef _SYSTEM_CHECK_BASE_HPP
#define _SYSTEM_CHECK_BASE_HPP

#include <rclcpp/rclcpp.hpp>

namespace system_health_check::base
{

/**
 * @brief Abstract base class for all system check plugins.
 * @details Any package that wants to perform a system check must inherit from this class
 *          and implement the check() and report() methods.
 */
class SystemCheckBase
{
public:
  virtual ~SystemCheckBase() {}

  /**
   * @brief Executes the system check with caching support.
   * @param node Shared pointer to the ROS 2 node.
   * @return true if the check passes, false otherwise.
   * @details This function follows the NVI pattern. It first checks if a valid result
   * exists in the cache for the generated ID. If found, it returns the cached
   * result immediately. Otherwise, it invokes `check_impl()` and saves the result.
   */
  bool check(rclcpp::Node::SharedPtr node)
  {
    prepare_check(node);

    // 1. Generate unique ID for this check instance
    std::string id = get_unique_id();

    // If ID is empty, disable caching and run normally
    if (id.empty()) {
      return check_impl(node);
    }

    // 2. Check Cache
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      if (results_cache_.count(id)) {
        cache_msg_ = " (Cached) " + reports_cache_[id];
        return results_cache_[id];
      }
    }

    // 3. Execute the actual check logic (implemented in derived classes)
    bool result = check_impl(node);

    // 4. Save result to Cache
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      results_cache_[id] = result;
      reports_cache_[id] = report(node);
    }

    return result;
  }

  /**
   * @brief Generates a human-readable report of the check.
   * @param node Shared pointer to the ROS 2 node.
   * @return A string containing diagnostic information.
   */
  std::string report(rclcpp::Node::SharedPtr node) { return report_impl(node) + cache_msg_; }

protected:
  /**
   * @brief Generates a unique identifier for the check instance.
   * @details Used as a key for the result cache.
   * @return A unique string ID (e.g., "ClassName::TargetName::Params").
   */
  virtual std::string get_unique_id() const { return ""; }

  /**
   * @brief Prepares the check before execution.
   * @param node Shared pointer to the ROS 2 node.
   * @details Derived classes can override this function to perform any necessary setup.
   */
  virtual void prepare_check([[maybe_unused]] rclcpp::Node::SharedPtr node) {};

  /**
   * @brief Actual implementation of the check logic.
   * @note Derived classes must implement this function instead of `check`.
   * @param node Shared pointer to the ROS 2 node.
   * @return true if the check passes, false otherwise.
   */
  virtual bool check_impl(rclcpp::Node::SharedPtr node) = 0;

  /**
   * @brief Generates a detailed report of the check.
   * @param node Shared pointer to the ROS 2 node.
   * @return std::string A human-readable report string.
   * @details It should return a string containing diagnostic information (e.g., sensor values, error codes).
   */
  virtual std::string report_impl(rclcpp::Node::SharedPtr node) = 0;

private:
  std::string cache_msg_ = "";

  // Static members to share results across different plugin instances
  inline static std::unordered_map<std::string, bool> results_cache_;
  inline static std::unordered_map<std::string, std::string> reports_cache_;
  inline static std::mutex cache_mutex_;
};  // namespace system_health_check

}  // namespace system_health_check::base

#endif  // !_SYSTEM_CHECK_BASE_HPP
