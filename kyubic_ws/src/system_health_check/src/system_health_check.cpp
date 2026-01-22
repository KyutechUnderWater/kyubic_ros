/**
 * @file system_health_check.cpp
 * @brief Check topic, service, action server, and other system
 * @author R.Ohnishi
 * @date 2026/01/21
 *
 * @details Topicの情報や各サーバが起動しているかなどのシステムをチェックする
 ***************************************************************************/

#include "system_health_check/system_health_check.hpp"

#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;

// colar ANSI escape squence
#define ANSI_RESET "\033[0m"
#define ANSI_BOLD_RED "\033[1;31m"
#define ANSI_BOLD_BLUE "\033[1;94m"
#define ANSI_LIGHT_GREY "\033[37m"
#define ANSI_NAME_STYLE "\033[106;90m"         // Background: light cyan, Text: black
#define ANSI_TAG_END "\033[0;96m\xee\x82\xb0"  // arrow grif: \xee\x82\xb0

namespace system_health_check
{

SystemCheck::SystemCheck(const rclcpp::NodeOptions & options) : Node("checker_node", options)
{
  this->declare_parameter("checks", std::vector<std::string>{});
  this->declare_parameter("details", false);

  try {
    loader_ = std::make_shared<pluginlib::ClassLoader<system_health_check::SystemCheckBase>>(
      "system_health_check", "system_health_check::SystemCheckBase");
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create ClassLoader: %s", ex.what());
  }

  init_timer_ = this->create_wall_timer(500ms, std::bind(&SystemCheck::run_checks, this));
}

void SystemCheck::run_checks()
{
  // Execute once
  init_timer_->cancel();

  auto check_plugins = this->get_parameter("checks").as_string_array();
  if (check_plugins.empty()) {
    RCLCPP_WARN(this->get_logger(), "No checks defined in parameters.");
    return;
  }

  bool all_passed = true;

  bool show_details = this->get_parameter("details").as_bool();
  std::vector<std::pair<std::string, std::string>> detailed_reports;  // (PluginName, ReportContent)

  RCLCPP_INFO(this->get_logger(), "\n");
  RCLCPP_INFO(this->get_logger(), "=== Check Start ===");

  for (const auto & plugin_name : check_plugins) {
    try {
      auto checker = loader_->createSharedInstance(plugin_name);
      std::string desc = loader_->getClassDescription(plugin_name);

      // Execute Check
      if (checker->check(this->shared_from_this())) {
        RCLCPP_INFO(
          this->get_logger(),
          ANSI_BOLD_BLUE "[PASS] " ANSI_NAME_STYLE "%s" ANSI_TAG_END ANSI_RESET ", %s",
          plugin_name.c_str(), desc.c_str());
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          ANSI_BOLD_RED "[FAIL] " ANSI_NAME_STYLE "%s" ANSI_TAG_END ANSI_RESET ", %s",
          plugin_name.c_str(), desc.c_str());
        all_passed = false;
      }

      // Output details if requested
      if (show_details) {
        std::string report_msg = checker->report(this->shared_from_this());
        detailed_reports.emplace_back(plugin_name, report_msg);
      }

    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Plugin error: %s", ex.what());
      all_passed = false;
    } catch (std::exception & ex) {
      RCLCPP_ERROR(
        this->get_logger(),
        ANSI_BOLD_RED "[FAIL] " ANSI_NAME_STYLE "%s" ANSI_TAG_END ANSI_RESET ANSI_BOLD_RED
                      " (Exception: %s)",
        plugin_name.c_str(), ex.what());
      all_passed = false;
    }
  }

  RCLCPP_INFO(this->get_logger(), "------------------------");
  if (all_passed) {
    RCLCPP_INFO(this->get_logger(), ANSI_BOLD_BLUE "ALL SYSTEM CHECKS PASSED");
  } else {
    RCLCPP_ERROR(this->get_logger(), ANSI_BOLD_RED "SYSTEM CHECK FAILED");
  }
  RCLCPP_INFO(this->get_logger(), "=======================");

  // --- Detailed Reports Output ---
  if (show_details && !detailed_reports.empty()) {
    RCLCPP_INFO(this->get_logger(), "\n");
    RCLCPP_INFO(this->get_logger(), "=== Detailed Report ===");

    for (const auto & item : detailed_reports) {
      // item.first: Plugin Name, item.second: Report Content
      RCLCPP_INFO(
        this->get_logger(), ANSI_NAME_STYLE "%s " ANSI_TAG_END ANSI_LIGHT_GREY ", %s" ANSI_RESET,
        item.first.c_str(), item.second.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "=======================");
  }

  // --- Node shutdown ---
  if (all_passed) {
    RCLCPP_INFO(this->get_logger(), "Checks finished successfully. Shutting down...");
    rclcpp::shutdown();  // Exit Code 0
  } else {
    RCLCPP_ERROR(this->get_logger(), "Checks failed. Exiting with failure status.");
    std::exit(EXIT_FAILURE);
  }
}

}  // namespace system_health_check

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(system_health_check::SystemCheck)
