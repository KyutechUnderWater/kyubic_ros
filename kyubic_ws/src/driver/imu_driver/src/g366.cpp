/**
 * @file g366.cpp
 * @brief G366 library
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details IMU(G366) のデータを取得
 * **************************************/

#include "imu_driver/g366.hpp"

#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <thread>

using namespace std::chrono_literals;

namespace imu_driver::g366
{

G366::G366(const char * _portname, const int _baudrate) : portname(_portname), baudrate(_baudrate)
{
  serial_ = std::make_shared<serial::Serial>(portname, baudrate);
}

bool G366::setup()
{
  // Check if ready
  while (!is_ready()) {
    std::cout << "Info [setup]: Is not ready" << std::endl;
    std::this_thread::sleep_for(1s);
  }

  // Reset
  software_reset();

  // Check hardware error
  uint16_t hard_error = self_test() & 0b0000000001100000;
  if (hard_error != 0) {
    std::cout << "Error [setup]: Hardware error" << std::endl;
    return false;
  }

  // Set config
  serial_->write(config_comm, sizeof(config_comm));

  // Set filter
  set_filter();

  // Set attitude motion profile
  set_atti_motion_profile();

  // Run a self-test and Ckeck diagnostic
  uint16_t test_stat = self_test();
  if (test_stat == 0) {
    std::cout << "Info [setup]: Self-test is clear" << std::endl;
  } else {
    std::cout << "Error [setup]: Self-test is faild. Error number is " << test_stat << std::endl;
    return false;
  }

  // Set sampling mode
  serial_->write(window0_select_wcomm, sizeof(window0_select_wcomm));
  serial_->write(sampling_mode_wcomm0, sizeof(sampling_mode_wcomm0));

  // Transient response measures. Wait 3 times.
  int count = 0;
  while (count < 3) {
    std::this_thread::sleep_for(10ms);
    if (update()) count++;
    std::cout << "Info [setup]: Transient response measures. Count(3) is " << count << std::endl;
  }

  return true;
}

bool G366::update()
{
  // Read data
  uint8_t buf[36];
  serial_->flush();
  serial_->write(burst_request_wcomm0, sizeof(burst_request_wcomm0));
  ssize_t len = serial_->read(buf, 36, 10ms);

  // Decord data
  if (len == 36 && buf[0] == 0x80 && buf[35] == 0x0d) {
    RAW_DATA_T raw_data_t;
    raw_data_t.head = buf[0];
    raw_data_t.flag = concat_8bit(buf[1], buf[2]);
    raw_data_t.temp = concat_8bit(buf[3], buf[4]);
    raw_data_t.x_gyro = concat_8bit(buf[5], buf[6]);
    raw_data_t.y_gyro = concat_8bit(buf[7], buf[8]);
    raw_data_t.z_gyro = concat_8bit(buf[9], buf[10]);
    raw_data_t.x_accl = concat_8bit(buf[11], buf[12]);
    raw_data_t.y_accl = concat_8bit(buf[13], buf[14]);
    raw_data_t.z_accl = concat_8bit(buf[15], buf[16]);
    raw_data_t.qtn0 = concat_8bit(buf[17], buf[18]);
    raw_data_t.qtn1 = concat_8bit(buf[19], buf[20]);
    raw_data_t.qtn2 = concat_8bit(buf[21], buf[22]);
    raw_data_t.qtn3 = concat_8bit(buf[23], buf[24]);
    raw_data_t.roll = concat_8bit(buf[25], buf[26]);
    raw_data_t.pitch = concat_8bit(buf[27], buf[28]);
    raw_data_t.yaw = concat_8bit(buf[29], buf[30]);
    raw_data_t.gpio = concat_8bit(buf[31], buf[32]);
    raw_data_t.count = concat_8bit(buf[33], buf[34]);
    raw_data_t.foot = buf[35];

    // LSB -> float
    data.meta.head = raw_data_t.head;
    data.meta.flag = raw_data_t.flag;
    data.meta.gpio = raw_data_t.gpio;
    data.meta.count = raw_data_t.count;
    data.meta.foot = raw_data_t.foot;
    data.temp = temp_sf * raw_data_t.temp + 25;
    data.x_gyro = gyro_sf * raw_data_t.x_gyro;
    data.y_gyro = gyro_sf * raw_data_t.y_gyro;
    data.z_gyro = gyro_sf * raw_data_t.z_gyro;
    data.x_accl = accel_sf * raw_data_t.x_accl;
    data.y_accl = accel_sf * raw_data_t.y_accl;
    data.z_accl = accel_sf * raw_data_t.z_accl;
    data.qtn0 = qtn_sf * raw_data_t.qtn0;
    data.qtn1 = qtn_sf * raw_data_t.qtn1;
    data.qtn2 = qtn_sf * raw_data_t.qtn2;
    data.qtn3 = qtn_sf * raw_data_t.qtn3;
    data.roll = atti_sf * raw_data_t.roll;
    data.pitch = atti_sf * raw_data_t.pitch;
    data.yaw = atti_sf * raw_data_t.yaw;

    // offset
    data.roll *= -1;

    if (data.pitch > 0) {
      data.pitch = (180.0 - data.pitch) * -1;
    } else if (data.pitch < 0) {
      data.pitch = 180.0 + data.pitch;
    }

    if (data.yaw > 0) {
      data.yaw = 180.0 - data.yaw;
    } else if (data.yaw < 0) {
      data.yaw = (180.0 + data.yaw) * -1;
    }

    return true;
  }

  std::cout << "Error reading" << std::endl;
  return false;
}

uint16_t G366::concat_8bit(uint8_t msb, uint8_t lsb)
{
  return static_cast<short>(msb << 8) | static_cast<short>(lsb);
}

void G366::set_filter(uint8_t filter_type)
{
  std::cout << "Info [set_filter]: Setting filter..." << std::endl;

  // Send set filter command
  unsigned char override_wcomm1[3];
  std::copy(filter_ctrl_wcomm1, filter_ctrl_wcomm1 + sizeof(filter_ctrl_wcomm1), override_wcomm1);
  override_wcomm1[1] = filter_type;
  serial_->write(window1_select_wcomm, sizeof(window1_select_wcomm));
  serial_->write(override_wcomm1, sizeof(override_wcomm1));

  // Wait set filter
  uint8_t stat_buf[4];
  bool running = true;
  do {
    // Wait 100ms
    std::this_thread::sleep_for(100ms);

    //Read the running status
    serial_->flush();
    serial_->write(filter_ctrl_rcomm1, sizeof(filter_ctrl_rcomm1));
    uint8_t len = serial_->read(stat_buf, sizeof(stat_buf), 10ms);

    if (len == 4) {
      running = stat_buf[2] & 0b00100000;
    } else {
      std::cout << "Error [set_filter]: Don't read runing status.";
    }
  } while (running);
  std::cout << "Info [set_filter]: Successful" << std::endl;
}

void G366::set_atti_motion_profile(uint8_t motion_type)
{
  std::cout << "Info [set_atti_motion_profile]: Setting attitude motion profile..." << std::endl;

  // Send set attitude motion profile command
  unsigned char override_wcomm1[3];
  std::copy(atti_motion_wcomm1, atti_motion_wcomm1 + sizeof(atti_motion_wcomm1), override_wcomm1);
  override_wcomm1[1] = motion_type;
  serial_->write(window1_select_wcomm, sizeof(window1_select_wcomm));
  serial_->write(override_wcomm1, sizeof(override_wcomm1));

  // Wait set profile
  uint8_t stat_buf[4];
  bool running = true;
  do {
    // Wait 100ms
    std::this_thread::sleep_for(100ms);

    //Read the running status
    serial_->flush();
    serial_->write(atti_motion_rcomm1, sizeof(atti_motion_rcomm1));
    uint8_t len = serial_->read(stat_buf, sizeof(stat_buf), 10ms);

    if (len == 4) {
      running = stat_buf[2] & 0b01000000;
    } else {
      std::cout << "Error [set_atti_motion_profile]: Don't read runing status.";
    }
  } while (running);
  std::cout << "Info [set_atti_motion_profile]: Successful" << std::endl;
}

uint16_t G366::diagnostic_status()
{
  std::cout << "Info [diagnostic_status]: Getting diagnostic status" << std::endl;

  // Read the result
  uint8_t result_buf[4];
  serial_->flush();
  serial_->write(window0_select_wcomm, sizeof(window0_select_wcomm));
  serial_->write(diag_stat_rcomm0, sizeof(diag_stat_rcomm0));
  uint8_t len = serial_->read(result_buf, sizeof(result_buf), 10ms);

  if (len == 4) {
    std::cout << "Info [diagnostic_status]: Successful" << std::endl;
    return concat_8bit(result_buf[1], result_buf[2]);
  } else {
    std::cout << "Error [diagnostic_status]: Don't read diag status.";
    return 65535;
  }
}

uint16_t G366::self_test()
{
  std::cout << "Info [self_test]: Start self-test" << std::endl;

  // Send self test command
  serial_->write(window1_select_wcomm, sizeof(window1_select_wcomm));
  serial_->write(self_test_wcomm1, sizeof(self_test_wcomm1));

  // Wait self test
  uint8_t stat_buf[4];
  bool running = true;
  do {
    // Wait 100ms
    std::this_thread::sleep_for(100ms);

    //Read the running status
    serial_->flush();
    serial_->write(self_test_rcomm1, sizeof(self_test_rcomm1));
    uint8_t len = serial_->read(stat_buf, sizeof(stat_buf), 10ms);

    if (len == 4) {
      running = stat_buf[1] & 0b00000100;
    } else {
      std::cout << "Error [self_test]: Don't read runing status." << std::endl;
    }
  } while (running);
  std::cout << "Info [self_test]: End self-test" << std::endl;

  // Confirm the result
  return diagnostic_status();
}

void G366::software_reset()
{
  serial_->flush();
  serial_->write(window1_select_wcomm, sizeof(window1_select_wcomm));
  serial_->write(software_reset_wcomm1, sizeof(software_reset_wcomm1));
  std::this_thread::sleep_for(800ms);
}

bool G366::is_ready()
{
  // Read the ready status
  uint8_t buf[4];
  serial_->flush();
  serial_->write(window1_select_wcomm, sizeof(window1_select_wcomm));
  serial_->write(check_ready_rcomm1, sizeof(check_ready_rcomm1));
  uint8_t len = serial_->read(buf, sizeof(buf), 10ms);

  uint8_t mask = 0b00000100;
  if (len == 4 && (buf[1] & mask) == false) {
    return true;
  } else {
    return false;
  }
}

std::shared_ptr<DATA> G366::get_data() { return std::make_shared<DATA>(data); }

// G366HWReseter::G366HWReseter(const char * _portname, const int _baudrate)
// : portname(_portname), baudrate(_baudrate)
// {
//   serial_ = std::make_shared<serial::Serial>(portname, baudrate);
// }
//
// void G366HWReseter::reset()
// {
//   serial_->write(&reset_comm, sizeof(reset_comm));
// }

}  // namespace imu_driver::g366
