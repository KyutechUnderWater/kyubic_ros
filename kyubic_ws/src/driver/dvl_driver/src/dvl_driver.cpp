/**
 * @file dvl_driver.cpp
 * @brief DVL driver
 * @author R.Ohnishi
 * @date 2024/10/30
 *
 * @details DVL(Path Finder) のデータを取得して，Topicを流す
 ************************************************************/

#include "dvl_driver/dvl_driver.hpp"

using namespace std::chrono_literals;

namespace dvl_driver
{

DVLDriver::DVLDriver() : Node("dvl_driver")
{
  // Get parameter from server
  address = this->declare_parameter("ip_address", "0.0.0.0");
  listener_port = this->declare_parameter("listener_port", 8888);
  sender_port = this->declare_parameter("sender_port", 8889);
  timeout = this->declare_parameter("timeout", 0);
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout);

  // Connect TCP
  listener_ = std::make_shared<path_finder::Listener>(address.c_str(), listener_port, 500);
  sender_ = std::make_shared<path_finder::Sender>(address.c_str(), sender_port, 500);
  RCLCPP_INFO(this->get_logger(), "DVL connection successful");

  if (!setup()) {
    exit(1);
  }

  // Create publisher & wall timer
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::DVL>("dvl", qos);
  timer_ = create_wall_timer(100ms, std::bind(&DVLDriver::update, this));
}

bool DVLDriver::setup()
{
  if (!sender_->break_cmd()) {
    RCLCPP_ERROR(this->get_logger(), "Failded to send the break command.");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Send break command.");
  return true;
}

bool DVLDriver::_update()
{
  if (!sender_->ping_cmd()) {
    RCLCPP_ERROR(this->get_logger(), "ping faild");
    return false;
  }

  if (!listener_->listen()) {
    RCLCPP_ERROR(this->get_logger(), "listen faild");
    return false;
  }

  return true;
}

void DVLDriver::update()
{
  auto msg = std::make_unique<driver_msgs::msg::DVL>();

  if (DVLDriver::_update()) {
    timeout_->reset(this->get_clock()->now());

    // Data acquisition
    std::shared_ptr<path_finder::Data> dvl_data_ = listener_->get_dvl_data();

    // Prepare message
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = "/pathfinder";

    msg->altitude = dvl_data_->altitude;

    msg->velocity.x = dvl_data_->x_vel_bottom;
    msg->velocity.y = dvl_data_->y_vel_bottom;
    msg->velocity.z = dvl_data_->z_vel_bottom;
    msg->velocity_error = dvl_data_->e_vel_bottom;

    // Set status based on error-velocity-bottom
    if (dvl_data_->e_vel_bottom == 0) {
      msg->status = driver_msgs::msg::DVL::STATUS_NORMAL;
    } else {
      msg->status = driver_msgs::msg::DVL::STATUS_WARNING;
    }

    RCLCPP_INFO(this->get_logger(), "Update DVL data, error_vel_btm: %d", msg->velocity_error);
  } else {
    // Error if timeout, otherwise warning and wait
    if (timeout_->check(this->get_clock()->now())) {
      msg->status = driver_msgs::msg::DVL::STATUS_ERROR;
      RCLCPP_ERROR(this->get_logger(), "Don't update DVL data.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed DVL data acquisition.");
      return;
    }
  }

  // Publish
  pub_->publish(std::move(msg));
}

}  // namespace dvl_driver

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<dvl_driver::DVLDriver>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
