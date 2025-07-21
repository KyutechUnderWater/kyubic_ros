/**
 * @file localization_component.cpp
 * @brief localization using dvl , imu, and depth
 * @author R.Ohnishi
 * @date 2025/05/30
 *
 * @details DVL，IMU，Depthセンサを用いた自己位置推定
 ****************************************************/

#include "localization/localization_component.hpp"

#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include <functional>
#include <future>
#include <memory>

using namespace std::chrono_literals;

namespace localization
{

Localization::Localization(const rclcpp::NodeOptions & options) : Node("localization", options)
{
  bool depth_enable = this->declare_parameter("depth", false);
  bool imu_enable = this->declare_parameter("imu", false);
  bool dvl_enable = this->declare_parameter("dvl", false);
  enabled_sensor |= depth_enable << 2 | imu_enable << 1 | (imu_enable ? dvl_enable : 0);

  odom_msg_ = std::make_shared<localization_msgs::msg::Odometry>();

  rclcpp::QoS qos(rclcpp::KeepLast(1));

  // Create publisher
  pub_ = create_publisher<localization_msgs::msg::Odometry>("odom", qos);

  // Create subscription
  sub_depth_ = create_subscription<localization_msgs::msg::Odometry>(
    "depth/odom", qos, std::bind(&Localization::depth_callback, this, std::placeholders::_1));
  sub_imu_ = create_subscription<localization_msgs::msg::Odometry>(
    "imu/transformed", qos, std::bind(&Localization::imu_callback, this, std::placeholders::_1));
  sub_dvl_ = create_subscription<localization_msgs::msg::Odometry>(
    "dvl/odom", qos, std::bind(&Localization::dvl_callback, this, std::placeholders::_1));

  // Create server
  srv_ = create_service<std_srvs::srv::Trigger>(
    "reset",
    std::bind(&Localization::reset_callback, this, std::placeholders::_1, std::placeholders::_2),
    qos, nullptr);

  // Create client
  client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_depth_ = create_client<std_srvs::srv::Trigger>("depth/reset", qos, client_cb_group_);
  client_imu_ = create_client<std_srvs::srv::Trigger>("imu/reset", qos, client_cb_group_);
  client_dvl_ = create_client<std_srvs::srv::Trigger>("dvl/reset", qos, client_cb_group_);

  // Create wall timer
  timer_ = create_wall_timer(100ms, std::bind(&Localization::publisher, this));
}

void Localization::depth_callback(const localization_msgs::msg::Odometry::UniquePtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Updated Depth odometry");
  all_updated |= 4;

  odom_msg_->header = msg->header;
  odom_msg_->status |= msg->status;
  odom_msg_->pose.position.z_depth = msg->pose.position.z_depth;
  odom_msg_->twist.linear.z_depth = msg->twist.linear.z_depth;
}

void Localization::imu_callback(const localization_msgs::msg::Odometry::UniquePtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Updated IMU transformed");
  all_updated |= 2;

  odom_msg_->header = msg->header;
  odom_msg_->status |= msg->status;
  odom_msg_->pose.orientation = msg->pose.orientation;
  odom_msg_->twist.angular = msg->twist.angular;
}

void Localization::dvl_callback(const localization_msgs::msg::Odometry::UniquePtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Updated DVL odometry");
  all_updated |= 1;

  this->odom_msg_->header = msg->header;
  odom_msg_->status |= msg->status;

  this->odom_msg_->pose.position.x = msg->pose.position.x;
  this->odom_msg_->pose.position.y = msg->pose.position.y;
  this->odom_msg_->pose.position.z_altitude = msg->pose.position.z_altitude;

  this->odom_msg_->pose.orientation = msg->pose.orientation;
  this->odom_msg_->twist.angular = msg->twist.angular;

  this->odom_msg_->twist.linear.x = msg->twist.linear.x;
  this->odom_msg_->twist.linear.y = msg->twist.linear.y;
  this->odom_msg_->twist.linear.z_altitude = msg->twist.linear.z_altitude;
}

void Localization::publisher()
{
  if (all_updated != enabled_sensor) {
    RCLCPP_WARN(this->get_logger(), "Don't update. Wait data comming.");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Updated localization");

  // Copy object (shared_ptr -> unique_ptr)
  auto msg = std::make_unique<localization_msgs::msg::Odometry>(*odom_msg_);
  pub_->publish(std::move(msg));

  // Reset flag
  all_updated = 0b11111000;
  odom_msg_->status = localization_msgs::msg::Odometry::STATUS_NORMAL;
}

void Localization::reset_callback(
  [[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr reqest,
  const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  int result = ~this->reset();

  // Check reset
  std::string error_msg = "";
  if (result & 1) error_msg += "DVL didn't reset\n";
  if ((result >> 1) & 1) error_msg += "IMU didn't reset\n";
  if ((result >> 2) & 1) error_msg += "Depth didn't reset\n";

  // Prepare response message
  if (error_msg == "") {
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "The reset was successful");
  } else {
    response->success = false;
    response->message = error_msg;
    RCLCPP_ERROR(this->get_logger(), "Failed to reset (%s)", error_msg.c_str());
  }
}

bool Localization::_is_server_active(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_)
{
  if (client_->wait_for_service(1s)) return true;

  //　In case of timeout
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
  } else {
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  return false;
}

FutureAndRequestId Localization::_reset_request(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_)
{
  // Send request
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  FutureAndRequestId result_future = client_->async_send_request(request).share();
  RCLCPP_INFO(this->get_logger(), "Send request to %s", client_->get_service_name());

  return result_future;
}

bool Localization::_reset_response(
  FutureAndRequestId future_, std::chrono::duration<long, std::ratio<1, 1000>> dulation,
  std::string service_name)
{
  std::future_status status = future_.wait_for(dulation);
  if (status == std::future_status::ready) {
    if (future_.get()->success) {
      RCLCPP_INFO(this->get_logger(), "The %s was successful!", service_name.c_str());
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "The %s failed!", service_name.c_str());
    }
  } else if (status == std::future_status::timeout) {
    RCLCPP_ERROR(this->get_logger(), "Timeout: Failed to call service '%s'", service_name.c_str());
  }
  return false;
}

int Localization::reset()
{
  FutureAndRequestId depth_future_, imu_future_, dvl_future_;
  if (_is_server_active(client_depth_)) depth_future_ = _reset_request(client_depth_);
  if (_is_server_active(client_imu_)) imu_future_ = _reset_request(client_imu_);
  if (_is_server_active(client_dvl_)) dvl_future_ = _reset_request(client_dvl_);

  int result = 0;
  result |= static_cast<int>(_reset_response(depth_future_, 5000ms, "depth/reset")) << 2;
  result |= static_cast<int>(_reset_response(imu_future_, 5000ms, "imu/reset")) << 1;
  result |= static_cast<int>(_reset_response(dvl_future_, 5000ms, "dvl/reset"));

  return result;
}

}  // namespace localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::Localization)
