/**
 * @file thruster_driver.cpp
 * @brief thruster driver
 * @author R.Ohnishi
 * @date 2024/10/30
 *
 * @details 各軸の推力をサブスクライブし，ESC制御マイコンに命令する
 *******************************************************************/

#include "thruster_driver/thruster_driver.hpp"

#include <cstring>
#include <iostream>

namespace thruster_driver
{

ThrusterDriver::ThrusterDriver() : Node("thruster_driver")
{
  portname = this->declare_parameter("serial_port", "/dev/ttyACM0");
  baudrate = this->declare_parameter("serial_speed", 115200);

  serial_ = std::make_shared<serial::Serial>(portname.c_str(), baudrate);
  RCLCPP_INFO(this->get_logger(), "Connected %s", portname.c_str());

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::Thruster>("thruster", qos);
  sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
    "robot_force", 10,
    std::bind(&ThrusterDriver::_robot_force_callback, this, std::placeholders::_1));
}

float ThrusterDriver::_calc_restrict_rate(float total, float limit)
{
  return limit / total;
}

void ThrusterDriver::_robot_force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  float force_x = msg->wrench.force.x * (-1);
  float force_y = msg->wrench.force.y * (-1);
  float force_z = msg->wrench.force.z * (-1);
  float moment_z = msg->wrench.torque.z;

  // TODO スラスタがサチュレーションしたときにheadingを優先する制御をいれる
  float thrusts[6];
  thrusts[0] = force_x / (4 * cos_h) - force_y / (4 * sin_h) +
               moment_z / (4 * (0.19 * cos_h + 0.14 * sin_h));  // HFR
  thrusts[1] = force_x / (4 * cos_h) + force_y / (4 * sin_h) -
               moment_z / (4 * (0.19 * cos_h + 0.14 * sin_h));  // HFL
  thrusts[4] = -force_x / (4 * cos_h) - force_y / (4 * sin_h) -
               moment_z / (4 * (0.19 * cos_h + 0.14 * sin_h));  // HRR
  thrusts[5] = -force_x / (4 * cos_h) + force_y / (4 * sin_h) +
               moment_z / (4 * (0.19 * cos_h + 0.14 * sin_h));  // HRL
  thrusts[2] = thrusts[3] = -force_z / (2 * cos_h);             // VL, VR

  float total_thrust = 0.0;
  for (int i = 0; i < 5; ++i) total_thrust += abs(thrusts[i]);

  if (total_thrust > max_thrust) {
    RCLCPP_WARN(
      this->get_logger(), "total_thrust(%f[N]) exceeds the max_thrust(%d[N]). Restrict thrust.",
      total_thrust, max_thrust);

    for (int i = 0; i < 6; ++i)
      thrusts[i] = thrusts[i] * _calc_restrict_rate(total_thrust, max_thrust);

    total_thrust = 0.0;
    for (int i = 0; i < 5; ++i) total_thrust += abs(thrusts[i]);
  }

  {
    // Send serial
    // format is like this *4.30;1.46;-0.00;-0.00;-1.46;-4.30#
    std::string s_buf = "";
    s_buf += start_char;
    for (int i = 0; i < 6; ++i) {
      if (i != 0) s_buf += delim_char;

      std::string s = std::to_string(thrusts[i]);
      s_buf += s.substr(0, s.size() - 4);
    }
    s_buf += end_char;

    const uint8_t * buf = reinterpret_cast<const uint8_t *>(s_buf.c_str());
    serial_->write(buf, strlen(s_buf.c_str()));
    RCLCPP_INFO(this->get_logger(), "sending message is %s", s_buf.c_str());
  }

  {
    // Publish thrust value
    auto msg = std::make_unique<driver_msgs::msg::Thruster>();
    msg->thrust_hfr.data = thrusts[0];
    msg->thrust_hrr.data = thrusts[4];
    msg->thrust_hrl.data = thrusts[5];
    msg->thrust_hfl.data = thrusts[1];
    msg->thrust_vmr.data = thrusts[3];
    msg->thrust_vml.data = thrusts[2];
    msg->total_thrust.data = total_thrust;

    pub_->publish(std::move(msg));
  }
}

}  // namespace thruster_driver

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<thruster_driver::ThrusterDriver>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
