/**
 * @file dvl_odometry.hpp
 * @brief Calculating position by accumulating DVL velocity
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details DVLで取得した速度を累積して，位置を算出
 *********************************************************/

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <localization/dvl_odometry_component.hpp>

using namespace std::chrono_literals;

namespace localization::dvl
{

DVLOdometry::DVLOdometry(const rclcpp::NodeOptions & options) : Node("dvl_odometry", options)
{
  offset[0] = this->declare_parameter("offset_x", 0.0);
  offset[1] = this->declare_parameter("offset_y", 0.0);
  offset[2] = this->declare_parameter("offset_z", 0.0);

  imu_msg_ = std::make_shared<localization_msgs::msg::Odometry>();

  rclcpp::QoS qos(rclcpp::KeepLast(1));

  pub_ = create_publisher<localization_msgs::msg::Odometry>("odom", qos);
  sub_imu_ = create_subscription<localization_msgs::msg::Odometry>(
    "transformed_imu", qos,
    std::bind(&DVLOdometry::update_imu_callback, this, std::placeholders::_1));
  sub_dvl_ = create_subscription<driver_msgs::msg::DVL>(
    "dvl", qos, std::bind(&DVLOdometry::update_callback, this, std::placeholders::_1));
  srv_ = create_service<std_srvs::srv::Trigger>(
    "reset",
    std::bind(&DVLOdometry::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

  this->reset();
}

void DVLOdometry::update_callback(const driver_msgs::msg::DVL::UniquePtr msg)
{
  auto odom_msg = std::make_unique<localization_msgs::msg::Odometry>();

  if (!msg->velocity_valid || imu_msg_->status.imu.id == common_msgs::msg::Status::ERROR) {
    RCLCPP_ERROR(this->get_logger(), "Don't calculate odometry. Because velocity error occurred");
    odom_msg->header = msg->header;
    odom_msg->status.dvl.id = common_msgs::msg::Status::ERROR;
    // 直前の有効値を保持したままpublishする(ゼロ値が新鮮な値であるかのように
    // 下流(localization_component等)へ伝播するのを防ぐ。Part A参照)。
    // pos_x/pos_yは常に有効(reset()の0初期化がdead-reckoning原点として正当な値)。
    odom_msg->pose.position.x = pos_x;
    odom_msg->pose.position.y = pos_y;
    // altitude/velocityは一度も有効値を受信していなければ0を「保持値」と称さない
    // (has_valid_measurement_==falseの間はodom_msgのデフォルト0のままにする。
    // status.dvl.id==ERRORなので下流はどのみち使わないが、この層単体でも
    // 「未計測」と「保持値」を混同しないようにする)。
    if (has_valid_measurement_) {
      odom_msg->pose.position.z_altitude = last_altitude_;
      odom_msg->twist.linear.x = last_velocity_world_.x();
      odom_msg->twist.linear.y = last_velocity_world_.y();
      odom_msg->twist.linear.z_altitude = last_velocity_world_.z();
    }
  } else {
    auto now = this->get_clock()->now();
    double dt = (now - pre_time).nanoseconds() * 1e-9;
    pre_time = now;

    tf2::Vector3 lever_arm(offset[0], offset[1], offset[2]);
    tf2::Vector3 vel_raw(msg->velocity.x, msg->velocity.y, msg->velocity.z);

    tf2::Vector3 angular_vel(
      imu_msg_->twist.angular.x * DEGREE_TO_RADIAN, imu_msg_->twist.angular.y * DEGREE_TO_RADIAN,
      imu_msg_->twist.angular.z * DEGREE_TO_RADIAN);

    tf2::Quaternion q_rot;
    q_rot.setRPY(
      imu_msg_->pose.orientation.x * DEGREE_TO_RADIAN,
      imu_msg_->pose.orientation.y * DEGREE_TO_RADIAN,
      imu_msg_->pose.orientation.z * DEGREE_TO_RADIAN);

    tf2::Vector3 vel_robot_body = vel_raw - angular_vel.cross(lever_arm);

    tf2::Vector3 vel_robot_world = tf2::quatRotate(q_rot, vel_robot_body);

    pos_x += vel_robot_world.x() * dt;
    pos_y += vel_robot_world.y() * dt;
    last_altitude_ = msg->altitude;
    last_velocity_world_ = vel_robot_world;
    has_valid_measurement_ = true;

    {
      odom_msg->header = msg->header;

      odom_msg->pose.position.x = pos_x;
      odom_msg->pose.position.y = pos_y;
      odom_msg->pose.position.z_altitude = msg->altitude;

      odom_msg->pose.orientation = imu_msg_->pose.orientation;
      odom_msg->twist.angular = imu_msg_->twist.angular;

      odom_msg->twist.linear.x = vel_robot_world.x();
      odom_msg->twist.linear.y = vel_robot_world.y();
      odom_msg->twist.linear.z_altitude = vel_robot_world.z();
    }
    RCLCPP_DEBUG(this->get_logger(), "Calculated DVL odometry");
  }
  pub_->publish(std::move(odom_msg));
}

void DVLOdometry::update_imu_callback(localization_msgs::msg::Odometry::UniquePtr msg)
{
  imu_msg_ = std::move(msg);
}

void DVLOdometry::reset_callback(
  [[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr request,
  const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  this->reset();
  RCLCPP_INFO(this->get_logger(), "Reset");

  response->success = true;
  response->message = "";
}

void DVLOdometry::reset()
{
  pre_time = this->get_clock()->now();
  pos_x = pos_y = 0.0;
  last_altitude_ = 0.0;
  last_velocity_world_ = tf2::Vector3(0.0, 0.0, 0.0);
  has_valid_measurement_ = false;
}

}  // namespace localization::dvl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::dvl::DVLOdometry)
