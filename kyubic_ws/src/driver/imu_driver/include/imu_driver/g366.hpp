/**
 * @file g366.hpp
 * @brief G366 library
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details IMU(G366) のデータを取得
 * **************************************/

#ifndef _G366_HPP
#define _G366_HPP

#include <memory>
#include <serial/serial.hpp>

/**
 * @namespace imu_driver
 * @brief For imu driver
 */
namespace imu_driver::g366
{

/**
 * @brief Store meta data of G366
 */
struct DATA_META
{
  unsigned char head;
  short flag;
  short gpio;
  short count;
  unsigned char foot;
};

/**
 * @brief Store data with modified type of G366
 */
struct DATA
{
  DATA_META meta;
  float temp;
  float x_gyro;
  float y_gyro;
  float z_gyro;
  float x_accl;
  float y_accl;
  float z_accl;
  float roll;
  float pitch;
  float yaw;
};

/**
 * @brief Store raw data of G355
 */
struct RAW_DATA_T
{
  unsigned char head;
  short flag;
  short temp;
  short x_gyro;
  short y_gyro;
  short z_gyro;
  short x_accl;
  short y_accl;
  short z_accl;
  short roll;
  short pitch;
  short yaw;
  short gpio;
  short count;
  unsigned char foot;
};

// TODO: 何のコマンドかを書く
/**
 * @brief Commands to configure the G366
 */
const unsigned char config_comm1[12] = {0xfe, 0x01, 0x0d, 0x0a, 0x00, 0x0d,
                                        0xfe, 0x00, 0x0d, 0x04, 0x00, 0x0d};
const unsigned char config_comm2[9] = {0xfe, 0x01, 0x0d, 0x86, 0x05, 0x0d, 0x06, 0x00, 0x0d};
const unsigned char config_comm3[3] = {0x06, 0x00, 0x0d};
const unsigned char config_comm4[15] = {0xfe, 0x01, 0x0d, 0x8A, 0x10, 0x0d, 0x0A, 0x00,
                                        0x0d, 0xfe, 0x00, 0x0d, 0x04, 0x00, 0x0d};
const unsigned char config_comm5[6] = {0xfe, 0x01, 0x0d, 0x8A, 0x80, 0x0d};
const unsigned char config_comm6[30] = {
  0xFE, 0x01, 0x0d, /* WINDOW = 1 */

  0x85, 0x09, 0x0d, /* 200 Sps */

  0x88, 0x00, 0x0d, /* UART Manual mode */

  0x8C, 0x06, 0x0d, /* GPIO = on, COUNT = on, CheckSum = off */

  0x8D, 0xF1, 0x0d, /* FLAG = on, TEMP = on, Gyro = on, ACCL = on, ATTI = on */

  0x8F, 0x00, 0x0d, /* TEMP = 16 bits, Gyro = 16 bits, ACCL = 16 bits, ATTI = 16 bits */

  0x93, 0x00, 0x0d, /* A_RANGE_CTRL 8G */

  0x95, 0x0c, 0x0d, /* eular mode, enable ATTI_ON */

  0xFE, 0x00, 0x0d, /* WINDOW = 0 */

  0x83, 0x01, 0x0d};
const unsigned char request_comm[3] = {0x80, 0x00, 0x0d};

/**
 * @brief G366 class
 */
class G366
{
public:
  /**
   * @brief Serial communication setting
   * @param portname Port name of serial device
   * @param baudrate Baudrate of serial communication
   * @details Setting such as port opening, and communication speed
   */
  explicit G366(const char * _portname, const int _baudrate);

  /**
   * @brief Setting G366 configuration
   */
  bool setup();

  /**
   * @brief Update g366 data
   * @details Aquitision of G366 data
   */
  bool update();

  /**
   * @brief Get G366 data
   * @return DATA structure of G366
   */
  std::shared_ptr<DATA> get_data();

private:
  const char * portname;
  const int baudrate;

  DATA data;

  std::shared_ptr<serial::Serial> serial_;
};

// class G366HWReseter
// {
// public:
//   /**
//    * @
//   */
//   explicit G366HWReseter(const char * _portname, const int _baudrate);
//
//   void reset();
//
// private:
//   const char * portname;
//   const int baudrate;
//
//   std::shared_ptr<serial::Serial> serial_;
//
//   const unsigned char reset_comm = 'r';
// };

}  // namespace imu_driver::g366

#endif
