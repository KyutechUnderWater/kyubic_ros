/**
 * @file imu_transform_component.cpp
 * @brief convert from imu frame to map frame
 * @author R.Ohnishi
 * @date 2025/05/30
 *
 * @details imuの座標系をワールド座標系に変換する
 ***********************************************/

#include "localization/imu_transform_component.hpp"

namespace localization
{

IMUTransform::IMUTransform(const rclcpp::NodeOptions & options) : Node("imu_transform", options)
{
  double x = Transform3D::deg2rad(this->declare_parameter("x", 0.0));
  double y = Transform3D::deg2rad(this->declare_parameter("y", 0.0));
  double z = Transform3D::deg2rad(this->declare_parameter("z", 0.0));
  rot_ = Transform3D::Rotation::fromRPY(x, y, z);

  rclcpp::QoS qos(rclcpp::KeepLast(1));

  pub_ = create_publisher<localization_msgs::msg::Odometry>("transformed", qos);
  sub_ = create_subscription<driver_msgs::msg::IMU>(
    "imu", qos, std::bind(&IMUTransform::update_callback, this, std::placeholders::_1));
  srv_ = create_service<std_srvs::srv::Trigger>(
    "reset",
    std::bind(&IMUTransform::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

  reset();
}

void IMUTransform::update_callback(driver_msgs::msg::IMU::UniquePtr msg)
{
  auto odom_msg = std::make_unique<localization_msgs::msg::Odometry>();

  if (msg->status == driver_msgs::msg::IMU::STATUS_ERROR) {
    RCLCPP_ERROR(this->get_logger(), "The imu data is invalid");
    odom_msg->status.imu = status_ = localization_msgs::msg::Status::ERROR;
  } else {
    msg = calc_euler(std::move(msg));
    msg = calc_quartenion(std::move(msg));

    status_ = localization_msgs::msg::Status::NORMAL;

    // Publish
    {
      odom_msg->header = msg->header;
      odom_msg->twist.angular = msg->gyro;
      odom_msg->pose.orientation.euler = msg->orient;
      odom_msg->pose.orientation.qtn = msg->qtn;
    }
    RCLCPP_DEBUG(this->get_logger(), "Calculated IMU transform");
  }
  pub_->publish(std::move(odom_msg));
}

driver_msgs::msg::IMU::UniquePtr IMUTransform::calc_euler(driver_msgs::msg::IMU::UniquePtr msg)
{
  // Copy current data
  gyro_ << msg->gyro.x, msg->gyro.y, msg->gyro.z;
  orient_ << msg->orient.x, msg->orient.y, msg->orient.z;

  Transform3D::Rotation ori =
    Transform3D::Rotation::fromRPY(msg->orient.x, msg->orient.y, msg->orient.z);
  // Transform
  Eigen::Vector3d gyro = rot_ * gyro_;
  Eigen::Vector3d orient = (rot_ * ori).toRotationMatrix().eulerAngles(2, 1, 0);

  // Add offset
  orient(0) -= offset_orient_(0);
  orient(1) -= offset_orient_(1);
  orient(2) -= offset_orient_(2);

  // yaw range
  // if (orient[2] < -180) orient[2] += 360;
  // if (180 < orient[2]) orient[2] -= 360;

  msg->gyro.x = gyro(0);
  msg->gyro.y = gyro(1);
  msg->gyro.z = gyro(2);
  msg->orient.x = orient(0);
  msg->orient.y = orient(1);
  msg->orient.z = orient(2);
  return msg;
}

driver_msgs::msg::IMU::UniquePtr IMUTransform::calc_quartenion(driver_msgs::msg::IMU::UniquePtr msg)
{
  qtn_ = Eigen::Quaterniond(msg->qtn.w, msg->qtn.x, msg->qtn.y, msg->qtn.z);

  // Transform
  // Eigen::Quaterniond q_relative = qtn_ * rot_.toQuaternion();
  // Eigen::Quaterniond q_relative = q * offset_qtn_;

  // TODO: 計算が正しいことを確認したら削除
  const double rad_to_deg = 180.0 / M_PI;
  // std::cout << "orient  roll: " << msg->orient.x << ", ";
  // std::cout << "  pitch: " << msg->orient.y << ", ";
  // std::cout << "  yaw: " << msg->orient.z << ", " << std::endl;
  // // Z-Y-X順のオイラー角に変換
  // Eigen::Vector3d euler = q_relative.toRotationMatrix().eulerAngles(2, 0, 1);
  // std::cout << "quart  Roll: " << euler.y() * rad_to_deg << ", ";
  // std::cout << "  Pitch: " << euler.z() * rad_to_deg << ", ";
  // std::cout << "  Yaw: " << euler.x() * rad_to_deg << std::endl;
  // std::cout << std::endl;
  // const double rad_to_deg = 180.0 / M_PI;
  std::cout << "orient  roll: " << orient_(0) << ", ";
  std::cout << "  pitch: " << orient_(1) << ", ";
  std::cout << "  yaw: " << orient_(2) << ", " << std::endl;
  // Z-Y-X順のオイラー角に変換
  Eigen::Vector3d euler = qtn_.toRotationMatrix().eulerAngles(2, 1, 0);
  std::cout << "quart   Roll: " << euler.z() * rad_to_deg << ", ";
  std::cout << "  Pitch: " << euler.y() * rad_to_deg << ", ";
  std::cout << "  Yaw: " << euler.x() * rad_to_deg << std::endl;
  std::cout << qtn_ << std::endl;
  std::cout << std::endl;

  // msg->qtn.w = q_relative.w();
  // msg->qtn.x = q_relative.x();
  // msg->qtn.y = q_relative.y();
  // msg->qtn.z = q_relative.z();
  return msg;
}

void IMUTransform::reset_callback(
  [[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr request,
  const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  bool result = this->reset();
  RCLCPP_INFO(this->get_logger(), "Reset: %s", result ? "success" : "faild");

  response->message = "";
  if (result)
    response->success = true;
  else
    response->success = false;
}

bool IMUTransform::reset()
{
  int count = 0;
  while (status_ == localization_msgs::msg::Status::ERROR) {
    std::this_thread::sleep_for(20ms);
    if (++count == 3) return false;
  };

  offset_orient_ = orient_;
  offset_qtn_ = qtn_.inverse();
  return true;
}

}  // namespace localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::IMUTransform)
