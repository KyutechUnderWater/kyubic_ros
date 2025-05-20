/**
 * @file dvl_odometry.hpp
 * @brief Calculating position by accumulating DVL velocity
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details DVLで取得した速度を累積して，位置を算出
 *********************************************************/

#include <dvl_driver/dvl_odometry_component.hpp>
#include <rclcpp/logging.hpp>

namespace dvl_driver
{

DVLOdometry::DVLOdometry(const rclcpp::NodeOptions & options) : Node("dvl_odometry", options)
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  pub_ = create_publisher<driver_msgs::msg::DVL>("dvl_odom", qos);
  sub_ = create_subscription<driver_msgs::msg::DVL>(
    "dvl", qos, std::bind(&DVLOdometry::_update_callback, this, std::placeholders::_1));

  this->reset();
}

void DVLOdometry::_update_callback(const driver_msgs::msg::DVL::UniquePtr msg)
{
  // return if velocity error, otherwise calculate odometry
  if (msg->velocity_error == 65536) {
    RCLCPP_ERROR(this->get_logger(), "Don't calculate odometry. Because velocity error occurred");
    return;
  }

  // Calculate update period (delta t)
  auto now = this->get_clock()->now();
  double dt = (now - pre_time).nanoseconds() / 1e9;
  pre_time = now;

  // Integral
  pos_x += msg->velocity.x * dt;
  pos_y += msg->velocity.y * dt;
  pos_z += msg->velocity.z * dt;

  // Publish
  {
    auto odom_msg = std::make_unique<driver_msgs::msg::DVL>();

    // copy msg
    odom_msg->header = msg->header;
    odom_msg->velocity_error = msg->velocity_error;
    odom_msg->velocity = msg->velocity;
    odom_msg->altitude = msg->altitude;

    // add position
    odom_msg->position.x = pos_x;
    odom_msg->position.y = pos_y;
    odom_msg->position.z = pos_z;

    pub_->publish(std::move(odom_msg));
  }
  RCLCPP_INFO(this->get_logger(), "Calculate odometry");
}

std::array<std::array<double, 3>, 3> DVLOdometry::get_odom()
{
  std::array<std::array<double, 3>, 3> odom = {{{vel_x, vel_y, vel_z}, {pos_x, pos_y, pos_z}}};
  return odom;
}

void DVLOdometry::reset()
{
  pre_time = this->get_clock()->now();
  pos_x = pos_y = pos_z = 0.0;
}

}  // namespace dvl_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dvl_driver::DVLOdometry)
