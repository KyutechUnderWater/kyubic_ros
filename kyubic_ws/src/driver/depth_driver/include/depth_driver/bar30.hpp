/**
 * @file bar30.hpp
 * @brief bar30 library
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details bar30のデータを取得
 **************************************************/

#ifndef _BAR30_HPP
#define _BAR30_HPP

#include <serial/serial.hpp>

#include <memory>

/**
 * @namespace depth_driver
 * @brief For depth driver
 */
namespace depth_driver
{
#pragma pack(push, 1)
struct DepthData
{
  unsigned int sequence;
  double depth;
};
#pragma pack(pop)

/**
 * @brief Bar30 class
 */
class Bar30
{
public:
  /**
   * @brief Serial communication setting
   * @param portname Port name of serial device
   * @param baudrate Baudrate of serial communication
   * @details Settings such as port opening and communication speed
   */
  explicit Bar30(const char * _portname, const int _baudrate);

  /**
   * @brief Update depth data
   * @return True if successful, False otherwise
   * @details Acquisition of depth data
   */
  bool update();

  /**
   * @brief Get depth data
   * @return depth data
   */
  DepthData get_data();

private:
  const char * portname;
  const int baudrate;

  std::shared_ptr<serial::Serial> serial_;

  DepthData data;
};

}  // namespace depth_driver
#endif
