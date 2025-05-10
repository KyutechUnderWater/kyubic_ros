/**
 * @file g366.cpp
 * @brief G366 library
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details IMU(G366) のデータを取得
 * **************************************/

#include "imu_driver/g366.hpp"

#include <iostream>

using namespace std::chrono_literals;

namespace imu_driver::g366
{

G366::G366(const char * _portname, const int _baudrate) : portname(_portname), baudrate(_baudrate)
{
  serial_ = std::make_shared<serial::Serial>(portname, baudrate);
}

void G366::setup()
{
  uint8_t buf[12];
  serial_->flush();
  serial_->write(config_comm1, sizeof(config_comm1));
  std::cout << "read" << std::endl;
  serial_->write(config_comm2, sizeof(config_comm2));
  serial_->read(buf, sizeof(buf), 10ms);

  while (buf[2] != 0x05) {
    serial_->write(config_comm3, sizeof(config_comm3));
    ssize_t len = serial_->read(buf, 4, 10ms);
    for (int i = 0; i < len; i++) printf("%02X ", buf[i]);
  }

  serial_->write(config_comm6, sizeof(config_comm6));
}

bool G366::update()
{
  serial_->flush();
  serial_->write(request_comm, sizeof(request_comm));

  uint8_t buf[28];
  ssize_t len = serial_->read(buf, 28, 10ms);

  if (len == 28 && buf[0] == 0x80 && buf[27] == 0x0d) {
    auto concat_bit = [](uint8_t msb, uint8_t lsb) -> short {
      return static_cast<short>(msb << 8) | static_cast<short>(lsb);
    };

    RAW_DATA_T raw_data_t;
    raw_data_t.head = buf[0];
    raw_data_t.flag = concat_bit(buf[1], buf[2]);
    raw_data_t.temp = concat_bit(buf[3], buf[4]);
    raw_data_t.x_gyro = concat_bit(buf[5], buf[6]);
    raw_data_t.y_gyro = concat_bit(buf[7], buf[8]);
    raw_data_t.z_gyro = concat_bit(buf[9], buf[10]);
    raw_data_t.x_accl = concat_bit(buf[11], buf[12]);
    raw_data_t.y_accl = concat_bit(buf[13], buf[14]);
    raw_data_t.z_accl = concat_bit(buf[15], buf[16]);
    raw_data_t.roll = concat_bit(buf[17], buf[18]);
    raw_data_t.pitch = concat_bit(buf[19], buf[20]);
    raw_data_t.yaw = concat_bit(buf[21], buf[22]);
    raw_data_t.gpio = concat_bit(buf[23], buf[24]);
    raw_data_t.count = concat_bit(buf[25], buf[26]);
    raw_data_t.foot = buf[27];

    data.meta.head = raw_data_t.head;
    data.meta.flag = raw_data_t.flag;
    data.meta.gpio = raw_data_t.gpio;
    data.meta.count = raw_data_t.count;
    data.meta.foot = raw_data_t.foot;
    data.temp = 0.00390625 * raw_data_t.temp + 25;
    data.x_gyro = (1 / 66.0) * raw_data_t.x_gyro;
    data.y_gyro = (1 / 66.0) * raw_data_t.y_gyro;
    data.z_gyro = (1 / 66.0) * raw_data_t.z_gyro;
    data.x_accl = (1 / 4.0) * raw_data_t.x_accl;
    data.y_accl = (1 / 4.0) * raw_data_t.y_accl;
    data.z_accl = (1 / 4.0) * raw_data_t.z_accl;
    data.roll = 0.00699411 * raw_data_t.roll;
    data.pitch = 0.00699411 * raw_data_t.pitch;
    data.yaw = 0.00699411 * raw_data_t.yaw;

    // TODO: 計算の確認
    if (data.yaw > 0)
      data.yaw = 180.0 - data.yaw;
    else if (data.yaw < 0)
      data.yaw = (180.0 + data.yaw) * -1;

    return true;
  }

  std::cout << "Error reading" << std::endl;
  return false;
}

std::shared_ptr<DATA> G366::get_data()
{
  return std::make_shared<DATA>(data);
}

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
