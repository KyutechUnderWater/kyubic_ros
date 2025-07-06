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

#include <serial/serial.hpp>

#include <memory>

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
  float qtn0;
  float qtn1;
  float qtn2;
  float qtn3;
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
  short qtn0;
  short qtn1;
  short qtn2;
  short qtn3;
  short roll;
  short pitch;
  short yaw;
  short gpio;
  short count;
  unsigned char foot;
};

/**
 * @brief Commands to configure the G366
 * @details wcomm is write command, rcomm is read command
 * comm0 is window 0 command, comm1 is windows 1 command
 */
using command = const unsigned char;
command window0_select_wcomm[3] = {0xFE, 0x00, 0x0d};  /// Select Window 0
command window1_select_wcomm[3] = {0xFE, 0x00, 0x0d};  /// Select Window 1

command burst_request_wcomm0[3] = {0x80, 0x00, 0x0d};   /// Burst command (BURST Mode). Read data
command config_mode_wcomm0[3] = {0x83, 0x00, 0x0d};     /// Select configuration mode
command sampling_mode_wcomm0[3] = {0x83, 0x01, 0x0d};   /// Select sampling mode
command diag_stat_rcomm0[3] = {0x04, 0x00, 0x0d};       /// Read Diagnostic status
command self_test_wcomm1[3] = {0x83, 0x04, 0x0d};       /// Self test command
command self_test_rcomm1[3] = {0x02, 0x00, 0x0d};       /// Read self test status
command software_reset_wcomm1[3] = {0x8A, 0x80, 0x0d};  /// Software reset command
command check_ready_rcomm1[3] = {0x0A, 0x00, 0x0d};     /// Read ready status
command filter_ctrl_wcomm1[3] = {0x86, 0x00, 0x0d};     /// Set Filter command
command filter_ctrl_rcomm1[3] = {0x06, 0x00, 0x0d};     /// Read set filter status
command atti_motion_wcomm1[3] = {0x86, 0x00, 0x0d};     /// Set attitude mortion profile
command atti_motion_rcomm1[3] = {0x06, 0x00, 0x0d};     /// Read set profile status

command config_comm[33] = {
  0xFE, 0x01, 0x0d,  /// WINDOW = 1
  0x85, 0x09, 0x0d,  /// 200 Sps
  0x88, 0x00, 0x0d,  /// UART Manual mode
  0x8C, 0x06, 0x0d,  /// GPIO = on, COUNT = on, CheckSum = off
  0x8D, 0xF3, 0x0d,  /// FLAG = on, TEMP = on, Gyro = on, ACCL = on, QTN = on, ATTI = on
  0x8F, 0x00, 0x0d,  /// TEMP = 16 bits, Gyro = 16 bits, ACCL = 16 bits, ATTI = 16 bits
  0x93, 0x00, 0x0d,  /// A_RANGE_CTRL 8G
  0x94, 0x00, 0x0d,  /// axis mode is XYZ-XYZ
  0x95, 0x0c, 0x0d,  /// eular mode, enable ATTI_ON
};
// const unsigned char config_comm1[12] = {0xfe, 0x01, 0x0d, 0x0a, 0x00, 0x0d,
//                                         0xfe, 0x00, 0x0d, 0x04, 0x00, 0x0d};
// const unsigned char config_comm2[9] = {0xfe, 0x01, 0x0d, 0x86, 0x05, 0x0d, 0x06, 0x00, 0x0d};
// const unsigned char config_comm3[3] = {0x06, 0x00, 0x0d};
// const unsigned char config_comm4[15] = {0xfe, 0x01, 0x0d, 0x8A, 0x10, 0x0d, 0x0A, 0x00,
//                                         0x0d, 0xfe, 0x00, 0x0d, 0x04, 0x00, 0x0d};
// const unsigned char config_comm5[6] = {0xfe, 0x01, 0x0d, 0x8A, 0x80, 0x0d};

/**
 * @brief The scale factor for each data
 */
const double temp_sf = 0.00699411;      /// for 16bit attitude data [degree celsius/LSB]
const double gyro_sf = 1.0 / 66.0;      /// for 16bit gyro data [(degree/s)/LSB]
const double accel_sf = 1.0 / 4.0;      /// for 16bit, output range ±8 acceleration data [(mG)/LSB]
const double qtn_sf = 1.0 / (1 << 14);  /// for 16bit quaternion data [-/LSB]
const double atti_sf = 0.00699411;      /// for 16bit attitude data [degree/LSB]

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
   * @brief Concatenate 8bit to 16bit
   */
  uint16_t concat_8bit(uint8_t msb, uint8_t lsb);

  /**
   * @brief Set filter
   * @param filter_type Value corresponding to the filter type. Default is 0b00001000.
   * @details 0b00001000 is 
   */
  void set_filter(uint8_t filter_type = 8);

  /**
   * @brief Set attitude motion profile
   * @param motion_type Value corresponding to the motion type. Default is 0b00100000.
   * @details 0b00100000 is modeC, which Assumed operating speed is 1 [m/s], Application examples are construction machinery.
   */
  void set_atti_motion_profile(uint8_t motion_type = 32);

  /**
   * @brief Run a self-test
   * @return diagnostic_status()
   */
  uint16_t self_test();

  /**
   * @brief Run a software reset
   */
  void software_reset();

  /**
   * @brief Check ready
   * @return true if ready, Otherwise is false.
   */
  bool is_ready();

  /**
   * @brief Get G366 data
   * @return DATA structure of G366
   */
  std::shared_ptr<DATA> get_data();

  /**
   * @brief Setting G366 configuration
   */
  bool setup();

  /**
   * @brief Update g366 data
   * @details Aquitision of G366 data
   */
  bool update();

private:
  const char * portname;
  const int baudrate;

  DATA data;

  std::shared_ptr<serial::Serial> serial_;

  /**
   * @brief Run a diagnostic
   * @return 65535 is faild to read diagnostic status, other is diagnostic status
   */
  uint16_t diagnostic_status();
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
