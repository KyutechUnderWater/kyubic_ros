/**
 * @file dvl_driver.hpp
 * @brief DVL driver
 * @author R.Ohnishi
 * @date 2024/10/30
 *
 * @details DVL(Path Finder) のデータを取得して，Topicを流す
 ************************************************************/

#include "dvl_driver/path_finder.hpp"

#include <rclcpp/rclcpp.hpp>

#include <driver_msgs/msg/path_finder.hpp>

/**
 * @namespace dvl_driver
 * @brief For dvl driver
 */
namespace dvl_driver
{

/**
 * @brief DVL driver class
 */
class DVLDriver : public rclcpp::Node
{
public:
  /**
   * @brief Connect Path Finder and Set Topic
   */
  explicit DVLDriver();

private:
  std::string address;
  int listener_port;
  int sender_port;

  std::shared_ptr<path_finder::Sender> sender_;
  std::shared_ptr<path_finder::Listener> listener_;

  rclcpp::Publisher<driver_msgs::msg::PathFinder>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Create Topic msgs from dvl data
   * @return PathFinder message
   */
  driver_msgs::msg::PathFinder::UniquePtr _create_msg(std::shared_ptr<path_finder::Data> dvl_data_);

  /**
   * @brief Get dvl data and Publish data
   * @return True if successful, False otherwise
   */
  bool update();
};

}  // namespace dvl_driver
