#pragma once
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <system_check/base_class/system_check_base.hpp>
#include <thread>

namespace system_check
{

/**
 * @brief Checks if a specific Service Server is available in the ROS graph.
 * Does not require message type templates.
 */
class ServiceServerCheckBase : public system_check::SystemCheckBase
{
public:
  virtual bool check(rclcpp::Node::SharedPtr node) override
  {
    if (service_name_.empty()) {
      status_msg_ = "Service name not set. Please invoke `set_config()` function.";
      return false;
    }

    auto start_time = std::chrono::steady_clock::now();
    auto timeout_duration = std::chrono::milliseconds(timeout_ms_);

    while (true) {
      // Get all available services in the ROS graph
      // returns std::map<std::string, std::vector<std::string>>
      auto service_map = node->get_service_names_and_types();

      // Check if the target service exists in the map keys
      if (service_map.find(service_name_) != service_map.end()) {
        status_msg_ = "Service server found.";
        return true;  // PASS
      }

      // Check timeout
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (elapsed > timeout_duration) {
        status_msg_ = "Timeout. Service server not found.";
        return false;  // FAIL
      }

      // Sleep briefly
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  std::string report([[maybe_unused]] rclcpp::Node::SharedPtr node) override
  {
    return "Service: " + service_name_ + " | " + status_msg_;
  }

  void set_config(const std::string service_name, const uint32_t timeout_ms)
  {
    service_name_ = service_name;
    timeout_ms_ = timeout_ms;
  };

private:
  std::string service_name_;
  uint32_t timeout_ms_ = 1000;
  std::string status_msg_;
};

}  // namespace system_check
