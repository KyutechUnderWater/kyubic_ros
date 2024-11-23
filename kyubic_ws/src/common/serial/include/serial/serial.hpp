/**
 * @file serial.hpp
 * @brief serial library
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details シリアル通信(UART)をするためのライブラリ
 **************************************************/

#ifndef _SERIAL_HPP
#define _SERIAL_HPP

#include <termios.h>

#include <chrono>
#include <string>

/**
 * @namespace serial
 * @brief For serial communication
 */
namespace serial {

/**
 * @brief serial communication class
 */
class Serial {
public:
  /**
   * @brief serial communication setting
   * @param portname port name of serial device
   * @param baudrate baudrate of serial communication
   * @details settings such as port opening and communication speed
   */
  explicit Serial(const char *portname, const int baudrate);

  /**
   * @brief port closing
   */
  ~Serial();

  /**
   * @brief write data via serial communication
   * @param buf writing data
   * @param len length of buf
   * @return none
   * @details write only len of data in buf
   */
  void write(const uint8_t *buffer, const size_t len);

  /**
   * @brief read data via serial communication
   * @param buf valiable to store the reading data
   * @param len bytes of reading data
   * @return ssize_t byte of read data
   */
  ssize_t read(uint8_t *buf, const size_t len);

  /**
   * @brief read data via serial communication, with timeout
   * @param buf valiable to store the reading data
   * @param len bytes of reading data
   * @param timeout timeout time
   * @return ssize_t byte of read data
   * @details wait until the specified number of bytes is received. However,
   * after the specified time has elapsed, it times out.
   */
  ssize_t read(uint8_t *buf, const size_t len,
               const std::chrono::duration<long, std::ratio<1, 1000>> timeout);

  /**
   * @brief read data until a terminating character is received, with timeout
   * @param buf valiable to store the reading data
   * @param len bytes of reading data
   * @param end_char character indicating the end of data
   * @param timeout timeout time
   * @return ssize_t byte of read data
   * @details wait until the specified character is received. However, after the
   * specified time has elapsed, it times out.
   */
  ssize_t read_until(uint8_t *buf, const size_t len, const char end_char,
                     std::chrono::duration<long, std::ratio<1, 1000>> timeout);

  /**
   * @brief configure serial communication settings
   * @param _baudrate baudrate of serial communication
   * @return none
   * @details settings such as baudrate, cflag, iflag and oflag
   */
  void setConfig(const int _baudrate);

  /**
   * @brief set termios settings
   * @return none
   * @details configure settings stored in tty
   */
  void setTermios();

  /**
   * @brief get termios settings
   * @return none
   * @details get termios settings and store in tty
   */
  void getTermios();

  /**
   * @brief flush buffer
   */
  void flush();

  int fd;

private:
  struct termios tty;
};

/**
 * @brief exception for serial class
 */
class SerialException : public std::exception {
public:
  /**
   * @brief generate of exception instance containing error messages
   * @param msg error messages
   */
  explicit SerialException(const std::string &msg) : _msg(msg) {}

  /**
   * @brief return error messages
   * @return char* error messages
   */
  const char *what() const noexcept { return _msg.c_str(); }

private:
  /// store error messages
  std::string _msg;
};

} // namespace serial

#endif
