/**
 * @file imu_driver.cpp
 * @brief imu driver
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details 深度データを取得し，トピックに流す
 * ********************************************/

#include "imu_driver/imu_driver.hpp"

#include <rclcpp/logging.hpp>

#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

// ROS2のノードとして、IMUのデータをシステム全体に配信する関数
namespace imu_driver
{

IMUDriver::IMUDriver() : Node("imu_driver")
{
  portname = this->declare_parameter("serial_port", "/dev/ttyACM0");
  baudrate = this->declare_parameter("serial_speed", 115200);
  timeout = this->declare_parameter("timeout", 0);
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout);

  g366_ = std::make_shared<g366::G366>(portname.c_str(), baudrate);
  RCLCPP_INFO(this->get_logger(), "Connected %s", portname.c_str());

  _setup();
  RCLCPP_INFO(this->get_logger(), "G366 settings successful");

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::IMU>("imu", qos);
  timer_ = create_wall_timer(10ms, std::bind(&IMUDriver::_update, this));
}

// TODO: Reset Nodeへの命令を実装
void IMUDriver::hw_reset()
{
  // g366_hw_resetter_ = std::make_shared<g366::G366HWReseter>(r_portname, r_baudrate);
  // g366_hw_resetter_->reset();
  rclcpp::sleep_for(1s);
}

void IMUDriver::_setup()
{
  hw_reset();
  if (!g366_->setup()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to Setup");
    exit(1);
  }
}

// IMUからデータを取得し、ROS 2メッセージに変換し、ネットワークに配信する
void IMUDriver::_update()
{
  auto msg = std::make_unique<driver_msgs::msg::IMU>();

  if (g366_->update()) {
    timeout_->reset(this->get_clock()->now());

    // Data acquisition
    std::shared_ptr<g366::DATA> data_ = g366_->get_data();

    // Prepare message
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = "imu";

    msg->status = driver_msgs::msg::IMU::STATUS_NORMAL;

    msg->temperature = data_->temp;
    msg->accel.x = data_->x_accl;
    msg->accel.y = data_->y_accl;
    msg->accel.z = data_->z_accl;
    msg->gyro.x = data_->x_gyro;
    msg->gyro.y = data_->y_gyro;
    msg->gyro.z = data_->z_gyro;
    msg->qtn.w = data_->qtn0;
    msg->qtn.x = data_->qtn1;
    msg->qtn.y = data_->qtn2;
    msg->qtn.z = data_->qtn3;
    msg->orient.x = data_->roll;
    msg->orient.y = data_->pitch;
    msg->orient.z = data_->yaw;

    RCLCPP_INFO(this->get_logger(), "Update imu data");
  } else {
    // Error if timeout, otherwise warning and wait
    if (timeout_->check(this->get_clock()->now())) {
      msg->status = driver_msgs::msg::IMU::STATUS_ERROR;

      RCLCPP_ERROR(
        this->get_logger(), "IMU driver timeout: %lu [ns]", timeout_->get_elapsed_time());
    } else {
      RCLCPP_WARN(this->get_logger(), "Don't update imu data");
      return;
    }
  }

  // Publish
  pub_->publish(std::move(msg));
}

}  // namespace imu_driver

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<imu_driver::IMUDriver>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
