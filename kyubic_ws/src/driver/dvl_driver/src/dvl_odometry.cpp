/**
 * @file dvl_odometry.hpp
 * @brief Calculating position by accumulating DVL velocity
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details DVLで取得した速度を累積して，位置を算出
 *********************************************************/

#include <localization/dvl_odometry.hpp>

#include <cstdint>

namespace localization
{

DVLOdometry::DVLOdometry() : Node("dvl_odometry")
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  sub_ = create_subscription<driver_msgs::msg::DVL>(
    "driver/dvl", qos, std::bind(&DVLOdometry::_update_callback, this, std::placeholders::_1));

  this->reset();
}

void DVLOdometry::_update_callback(const driver_msgs::msg::DVL::UniquePtr msg)
{
  // TODO:msgの有効性チェック
  auto now = this->get_clock()->now();
  uint64_t dt = (now - pre_time).nanoseconds();
  pre_time = now;

  vel_x = msg->velocity.x;
  vel_y = msg->velocity.y;
  vel_z = msg->velocity.z;

  // Integral
  pos_x += vel_x * dt;
  pos_y += vel_y * dt;
  pos_z += vel_z * dt;
}

std::array<std::array<double, 3>, 3> DVLOdometry::get_odom()
{
  std::array<std::array<double, 3>, 3> odom = {{{vel_x, vel_y, vel_z}, {pos_x, pos_y, pos_z}}};
  return odom;
}

void DVLOdometry::reset()
{
  pre_time = this->get_clock()->now();
  vel_x = vel_y = vel_z = 0.0;
  pos_x = pos_y = pos_z = 0.0;
}

}  // namespace localization
