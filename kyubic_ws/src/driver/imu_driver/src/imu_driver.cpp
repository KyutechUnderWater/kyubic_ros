/**
 * @file imu_driver.cpp
 * @brief imu driver
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details 深度データを取得し，トピックに流す
 * ********************************************/

#include "imu_driver/imu_driver.hpp"

using namespace std::chrono_literals;

namespace imu_driver
{

IMUDriver::IMUDriver() : Node("imu_driver")
{
  portname = this->declare_parameter("serial_port", "/dev/ttyACM0");
  baudrate = this->declare_parameter("serial_speed", 115200);

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
  g366_->setup();
}

void IMUDriver::_update()
{
  if (g366_->update()) {
    std::shared_ptr<g366::DATA> data_ = g366_->get_data();

    auto msg = std::make_unique<driver_msgs::msg::IMU>();
    msg->gyro.x = data_->x_gyro;
    msg->gyro.y = data_->y_gyro;
    msg->gyro.z = data_->z_gyro;
    msg->accel.x = data_->x_accl;
    msg->accel.y = data_->y_accl;
    msg->accel.z = data_->z_accl;
    msg->orient.x = data_->roll;
    msg->orient.y = data_->pitch;
    msg->orient.z = data_->yaw;

    pub_->publish(std::move(msg));
    RCLCPP_INFO(this->get_logger(), "Update imu data");
  } else {
    RCLCPP_WARN(this->get_logger(), "Don't update imu data");
  }
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
