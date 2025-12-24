/**
 * @file dvl_setting.hpp
 * @brief DVL Configuration Client Node
 * @date 2025/07/05
 */

#ifndef DVL_SETTING_HPP_
#define DVL_SETTING_HPP_

#include <driver_msgs/srv/command.hpp>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace dvl_driver
{

class DVLSetting : public rclcpp::Node
{
public:
  DVLSetting();
  ~DVLSetting();

  /**
   * @brief Main routine to read config file and apply settings via Service
   * @return true if all commands succeeded
   */
  bool configure();

private:
  // ROS Service Client
  rclcpp::Client<driver_msgs::srv::Command>::SharedPtr client_;

  // Parameters
  std::string config_file_path_;

  // Helpers
  std::vector<std::string> load_config_file(const std::string & path);

  // Service wrapper (Synchronous-like)
  std::string call_service(const std::string & command);

  // Command logic
  bool execute_command(const std::string & cmd);
  std::string get_query_command(const std::string & cmd);
  std::string trim(const std::string & str);
};

}  // namespace dvl_driver

#endif  // DVL_SETTING_HPP_
