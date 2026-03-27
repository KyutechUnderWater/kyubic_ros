/**
 * @file localization_component.cpp
 * @brief localization using dvl , imu, and depth
 * @author R.Ohnishi
 * @date 2025/05/30
 *
 * @details DVL，IMU，Depthセンサを用いた自己位置推定
 ****************************************************/

#include "localization/localization_component.hpp"

using namespace std::chrono_literals;

namespace localization
{

Localization::Localization(const rclcpp::NodeOptions & options) : Node("localization", options)
{
  depth_enable_ = this->declare_parameter("depth", false);
  imu_enable_ = this->declare_parameter("imu", false);
  dvl_enable_ = this->declare_parameter("dvl", false);
  gnss_enable_ = this->declare_parameter("gnss", false);

  odom_depth_ = std::make_shared<localization_msgs::msg::Odometry>();
  odom_msg_ = std::make_shared<localization_msgs::msg::Odometry>();

  rclcpp::QoS qos(rclcpp::KeepLast(1));

  // Create publisher
  pub_ = create_publisher<localization_msgs::msg::Odometry>("odom", qos);

  // Create subscriber
  if (depth_enable_) {
    enabled_sensors_ |= 0b00000001;
    sub_depth_ = create_subscription<localization_msgs::msg::Odometry>(
      "depth/odom", qos, std::bind(&Localization::depth_callback, this, std::placeholders::_1));
  }

  if (gnss_enable_) {
    enabled_sensors_ |= 0b00001110;
    sub_gnss_ = create_subscription<localization_msgs::msg::Odometry>(
      "gnss/global_pos", qos, std::bind(&Localization::gnss_callback, this, std::placeholders::_1));
  } else if (dvl_enable_) {
    enabled_sensors_ |= 0b00000110;
    sub_dvl_ = create_subscription<localization_msgs::msg::Odometry>(
      "dvl/odom", qos, std::bind(&Localization::dvl_callback, this, std::placeholders::_1));
  } else if (imu_enable_) {
    enabled_sensors_ |= 0b00000010;
    sub_imu_ = create_subscription<localization_msgs::msg::Odometry>(
      "imu/transformed", qos, std::bind(&Localization::imu_callback, this, std::placeholders::_1));
  }

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
  client_gnss_ = create_client<std_srvs::srv::Trigger>("gnss/reset", qos, client_cb_group_);

  // Create wall timer
  timer_ = create_wall_timer(100ms, std::bind(&Localization::publisher, this));
}

void Localization::depth_callback(const localization_msgs::msg::Odometry::UniquePtr msg)
{
  odom_depth_->header = msg->header;
  odom_depth_->status.depth.id = msg->status.depth.id;
  odom_depth_->pose.position.z_depth = msg->pose.position.z_depth;
  odom_depth_->twist.linear.z_depth = msg->twist.linear.z_depth;

  updated_ |= 0b00000001;
  RCLCPP_DEBUG(this->get_logger(), "Updated Depth odometry");
}

void Localization::imu_callback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  odom_msg_ = msg;
  updated_ |= 0b00000010;
  RCLCPP_DEBUG(this->get_logger(), "Updated IMU transformed");
}

void Localization::dvl_callback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  odom_msg_ = msg;
  updated_ |= 0b00000110;
  RCLCPP_DEBUG(this->get_logger(), "Updated DVL odometry");
}

void Localization::gnss_callback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  odom_msg_ = msg;
  updated_ |= 0b00001110;
  RCLCPP_DEBUG(this->get_logger(), "Updated DVL odometry");
}

void Localization::publisher()
{
  if (updated_ != enabled_sensors_) {
    RCLCPP_DEBUG(this->get_logger(), "Don't update. Wait data comming.");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Updated localization");

  // Copy object (shared_ptr -> unique_ptr)
  auto odom_msg = std::make_unique<localization_msgs::msg::Odometry>(*odom_msg_);

  // Copy depth data
  if (depth_enable_) {
    odom_msg->status.depth.id = odom_depth_->status.depth.id;
    odom_msg->pose.position.z_depth = odom_depth_->pose.position.z_depth;
    odom_msg->twist.linear.z_depth = odom_depth_->twist.linear.z_depth;
  }

  pub_->publish(std::move(odom_msg));

  // Reset flag
  updated_ = 0;
}

void Localization::reset_callback(
  [[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr request,
  const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  int result = ~this->reset();

  // Check reset
  std::string error_msg = "";
  if (result & 1) error_msg += "Depth didn't reset\n";
  if ((result >> 1) & 1) error_msg += "IMU didn't reset\n";
  if ((result >> 2) & 1) error_msg += "DVL didn't reset\n";
  if ((result >> 3) & 1) error_msg += "GNSS didn't reset\n";

  // Prepare response message
  if (error_msg == "") {
    response->success = true;
    response->message = "Localization reset successfully";
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
  // Check future empty
  if (!future_.valid()) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid future: %s request was not sent.", service_name.c_str());
    return false;
  }

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
  // Define a struct to group the target sensor information for the reset
  struct ResetTarget
  {
    bool is_enabled;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
    const char * name;
    int bit_shift;
    FutureAndRequestId future;
  };

  // Create an array of configurations for each sensor
  ResetTarget targets[] = {
    {depth_enable_, client_depth_, "depth/reset", 0, {}},
    {imu_enable_, client_imu_, "imu/reset", 1, {}},
    {dvl_enable_, client_dvl_, "dvl/reset", 2, {}},
    {gnss_enable_, client_gnss_, "gnss/reset", 3, {}}};

  // Send reset requests in parallel only to the enabled sensors
  for (auto & target : targets) {
    if (target.is_enabled && _is_server_active(target.client)) {
      target.future = _reset_request(target.client);
    }
  }

  int result = 0;

  // Collect results and perform bitwise operations
  for (auto & target : targets) {
    if (!target.is_enabled) {
      result |= (1 << target.bit_shift);
    } else {
      // Wait for the response and reflect the success/failure in the specific bit
      bool success = _reset_response(target.future, 1000ms, target.name);
      result |= (static_cast<int>(success) << target.bit_shift);
    }
  }

  return result;
}

}  // namespace localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::Localization)
