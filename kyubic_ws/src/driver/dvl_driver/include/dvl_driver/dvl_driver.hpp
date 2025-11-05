/**
 * @file dvl_driver.hpp
 * @brief DVL driver
 * @author R.Ohnishi
 * @date 2024/10/30
 *
 * @details DVL(Path Finder) のデータを取得して，Topicを流す
 ************************************************************/

#include <driver_msgs/msg/dvl.hpp>
#include <driver_msgs/srv/command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <timer/timeout.hpp>

#include "dvl_driver/path_finder.hpp"

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
  ~DVLDriver();

private:
  std::string address;
  int listener_port;
  int sender_port;
  uint64_t timeout;

  const std::string CRCF = "\r\n";
  bool command_mode = false;
  std::shared_ptr<timer::Timeout> timeout_;

  std::shared_ptr<path_finder::Sender> sender_;
  std::shared_ptr<path_finder::Listener> listener_;

  rclcpp::Publisher<driver_msgs::msg::DVL>::SharedPtr pub_;
  rclcpp::Service<driver_msgs::srv::Command>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Send a break command for startup
   * @return True if successful, False otherwise
   */
  bool setup();

  /**
   * @brief Get dvl data
   * @return True if successful, False otherwise
   */
  bool _update();

  /**
   * @brief Publish data
   */
  void update();

  void sendCommandCallback(
    const driver_msgs::srv::Command::Request::SharedPtr request,
    driver_msgs::srv::Command::Response::SharedPtr response);
};

}  // namespace dvl_driver
