/**
 * @file system_health_check_base.hpp
 * @brief Abstruct Class of check process
 * @author R.Ohnishi
 * @date 2026/01/21
 *
 * @details 各種チェック処理のベースとなるクラス
 **********************************************/

#ifndef _SYSTEM_CHECK_BASE_HPP
#define _SYSTEM_CHECK_BASE_HPP

#include <rclcpp/rclcpp.hpp>

namespace system_health_check
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
    * @brief Virtual function to execute the check.
    * @param node Shared pointer to the ROS 2 node.
    * @return true: PASS, false: FAIL
    */
  virtual bool check(rclcpp::Node::SharedPtr node) = 0;

  /**
    * @brief Generates a detailed report of the check.
    * @param node Shared pointer to the ROS 2 node.
    * @return std::string A human-readable report string.
    * @details It should return a string containing diagnostic information (e.g., sensor values, error codes).
    */
  virtual std::string report(rclcpp::Node::SharedPtr node) = 0;
};

}  // namespace system_health_check

#endif  // !_SYSTEM_CHECK_BASE_HPP
