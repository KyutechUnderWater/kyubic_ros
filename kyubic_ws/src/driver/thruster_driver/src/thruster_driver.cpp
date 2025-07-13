/**
 * @file thruster_driver.cpp
 * @brief thruster driver
 * @author R.Ohnishi
 * @date 2024/10/30
 *
 * @details 各軸の推力をサブスクライブし，ESC制御マイコンに命令する
 *******************************************************************/

#include "thruster_driver/thruster_driver.hpp"

#include <cassert>
#include <cstring>
#include <iostream>
#include <numeric>

namespace thruster_driver
{

ThrusterDriver::ThrusterDriver() : Node("thruster_driver")
{
  portname = this->declare_parameter("serial_port", "/dev/ttyACM0");
  baudrate = this->declare_parameter("serial_speed", 115200);
  max_thrust = this->declare_parameter("max_thrust", 80);
  max_thrust_per = this->declare_parameter("max_thrust_per", 30);

  // Check value
  assert(0.0 < max_thrust && 0.0 < max_thrust_per);

  serial_ = std::make_shared<serial::Serial>(portname.c_str(), baudrate);
  RCLCPP_INFO(this->get_logger(), "Connected %s", portname.c_str());

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<driver_msgs::msg::Thruster>("thruster", qos);
  sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
    "robot_force", 10,
    std::bind(&ThrusterDriver::robot_force_callback, this, std::placeholders::_1));
}

std::array<float, NUM_THRUSTERS> ThrusterDriver::_wrench2thrusts(
  float f_x, float f_y, float f_z, float t_x, float t_z)
{
  // Individual forces and torque
  float thrust_x = f_x * f_x_scale;
  float thrust_y = f_y * f_y_scale;
  float thrust_z = f_z * f_z_scale;
  float thrust_x_rot = t_x * t_x_scale;
  float thrust_z_rot = t_z * t_z_scale;

  // Calculate thrusts
  std::array<float, NUM_THRUSTERS> thrusts;
  thrusts.at(0) = -thrust_x + thrust_y + thrust_z_rot;  // HFR
  thrusts.at(1) = -thrust_x - thrust_y - thrust_z_rot;  // HFL
  thrusts.at(4) = thrust_x + thrust_y - thrust_z_rot;   // HRR
  thrusts.at(5) = thrust_x - thrust_y + thrust_z_rot;   // HRL
  thrusts.at(2) = thrust_z - thrust_x_rot;              // VL
  thrusts.at(3) = thrust_z + thrust_x_rot;              // VR
  return thrusts;
}

float ThrusterDriver::_restrict_thrust(std::array<float, NUM_THRUSTERS> thrusts)
{
  auto calc_total_thrust = [](const std::array<float, NUM_THRUSTERS> thrusts) {
    return std::accumulate(
      thrusts.begin(), thrusts.end(), 0.0, [](int acc, int i) { return acc + abs(i); });
  };

  // If total_thrust > max_thrust, Restrict thrust
  float total_thrust = calc_total_thrust(thrusts);
  if (total_thrust > max_thrust) {
    RCLCPP_WARN(
      this->get_logger(), "total_thrust(%f[N]) exceeds the max_thrust(%f[N]). Restrict thrust.",
      total_thrust, max_thrust);

    float scale = max_thrust / total_thrust;
    for (size_t i = 0; i < thrusts.size(); i++) thrusts[i] *= scale;
  }

  // Individual thruster constraints
  for (size_t i = 0; i < thrusts.size(); i++) {
    if (abs(thrusts[i]) > max_thrust_per) {
      RCLCPP_WARN(
        this->get_logger(),
        "thrusts[%lu](%f[N]) exceeds the max_thrust_per(%f[N]). Restrict thrust.", i, thrusts[i],
        max_thrust_per);
      thrusts[i] = std::clamp(thrusts[i], -max_thrust_per, max_thrust_per);
    }
  }
  return calc_total_thrust(thrusts);
}

void ThrusterDriver::robot_force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  float force_x = msg->wrench.force.x;
  float force_y = msg->wrench.force.y;
  float force_z = msg->wrench.force.z;
  float moment_x = msg->wrench.torque.x;
  float moment_z = msg->wrench.torque.z;

  // Calculate thrust
  std::array<float, 6> thrusts = _wrench2thrusts(force_x, force_y, force_z, moment_x, moment_z);

  // Limitation
  float total_thrust = _restrict_thrust(thrusts);

  {
    // Send serial
    // format is like this *4.30;1.46;-0.00;-0.00;-1.46;-4.30#
    std::string s_buf = "";
    s_buf += start_char;
    for (int i = 0; i < 6; ++i) {
      if (i != 0) {
        s_buf += delim_char;
      }

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
    msg->thrust_hfr = thrusts[0];
    msg->thrust_hrr = thrusts[4];
    msg->thrust_hrl = thrusts[5];
    msg->thrust_hfl = thrusts[1];
    msg->thrust_vmr = thrusts[3];
    msg->thrust_vml = thrusts[2];
    msg->total_thrust = total_thrust;

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
