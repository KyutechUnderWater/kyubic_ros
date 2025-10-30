/**
 * @file serial.cpp
 * @brief serial library
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details シリアル通信(UART)をするためのライブラリ
 **************************************************/

#include "serial/serial.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <future>

using namespace std::chrono_literals;

namespace serial
{

Serial::Serial(const char * portname, const int baudrate)
{
  // O_RDWR: Read-Write
  // O_NOCTTY: ノイズ等による不意のctrl+cを防ぐため、tty制御をなしに
  fd = open(portname, O_RDWR | O_NOCTTY);  // nessesary O_SYNC?
  if (fd < 0) {
    std::string s = (std::string)portname + " doesn't open: " + strerror(errno);
    throw SerialException(s);
  }

  setConfig(baudrate);
}

Serial::~Serial() { close(fd); }

void Serial::setConfig(const int _baudrate)
{
  int baudrate = 0;
  switch (_baudrate) {
    case 9600:
      baudrate = B9600;
      break;
    case 19200:
      baudrate = B19200;
      break;
    case 38400:
      baudrate = B38400;
      break;
    case 57600:
      baudrate = B57600;
      break;
    case 115200:
      baudrate = B115200;
      break;
    case 230400:
      baudrate = B230400;
      break;
    case 460800:
      baudrate = B460800;
      break;
    case 500000:
      baudrate = B500000;
      break;
    case 921600:
      baudrate = B921600;
      break;
    case 1000000:
      baudrate = B1000000;
      break;
    case 1500000:
      baudrate = B1500000;
      break;
    case 2000000:
      baudrate = B2000000;
      break;
    default:
      std::string s = std::string("Unknown baud rate ") + std::to_string(_baudrate);
      throw SerialException(s);
  }

  getTermios();

  // set baudrate
  cfsetispeed(&tty, baudrate);
  cfsetospeed(&tty, baudrate);

  tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
  tty.c_cflag &= ~PARENB;          /* no parity bit*/
  tty.c_cflag &= ~CSTOPB;          /* only need 1 stop bit */
  tty.c_cflag &= ~CSIZE;           /* set data size */
  tty.c_cflag &= ~CRTSCTS;         /* no hardware flowcontrol */
  tty.c_cflag |= CS8;              /* 8-bit characters */

  // set non-canonical mode
  tty.c_iflag &=
    ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
  tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG);
  tty.c_oflag &= ~OPOST;

  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 5; /* Wait for up to 5 deciseconds */
  tty.c_cc[VMIN] = 0;

  setTermios();
}

void Serial::getTermios()
{
  if (tcgetattr(fd, &tty) < 0) {
    std::string s = std::string("Error from tcgetattr: ") + strerror(errno);
    throw SerialException(s);
  }
}

void Serial::setTermios()
{
  if (tcsetattr(fd, TCSANOW, &tty) < 0) {
    std::string s = std::string("Error from tcsetattr: ") + strerror(errno);
    throw SerialException(s);
  }
}

void Serial::write(const uint8_t * buf, const size_t len)
{
  size_t written_len = ::write(fd, buf, len);
  if (written_len != len) {
    std::string s = std::string("Error writing: ") + strerror(errno);
    throw SerialException(s);
  }
}

ssize_t Serial::read(uint8_t * buf, const size_t len)
{
  ssize_t read_len = ::read(fd, buf, len);
  if (read_len < 0) {
    std::string s = std::string("Error reading: ") + strerror(errno);
    throw SerialException(s);
  }
  return read_len;
}

ssize_t Serial::read(
  uint8_t * buf, const size_t len, const std::chrono::duration<long, std::ratio<1, 1000>> timeout)
{
  if (len == 0) return 0;

  std::atomic<bool> end_flag(false);
  std::future<ssize_t> future = std::async(std::launch::async, [this, buf, len, &end_flag]() {
    ssize_t read_len = 0;
    while (!end_flag.load(std::memory_order_relaxed)) {
      ssize_t newly_read = read(buf + read_len, len - read_len);
      if (newly_read > -1) {
        read_len += newly_read;
        if (static_cast<size_t>(read_len) >= len) {
          return read_len;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return read_len;
  });

  std::future_status status = future.wait_for(timeout);
  end_flag.store(true, std::memory_order_relaxed);
  if (status == std::future_status::ready) {
    return future.get();
  } else {
    return (ssize_t)0;
  }
}

ssize_t Serial::read_until(
  uint8_t * buf, const size_t len, const char end_char,
  std::chrono::duration<long, std::ratio<1, 1000>> timeout)
{
  if (len == 0) return 0;

  std::atomic<bool> end_flag(false);
  std::future<ssize_t> future =
    std::async(std::launch::async, [this, buf, len, &end_char, &end_flag]() {
      ssize_t read_len = 0;
      while (!end_flag.load(std::memory_order_relaxed)) {
        size_t rl = read(buf + read_len, 1);
        read_len += rl;
        if (buf[read_len - 1] == (uint8_t)end_char) {
          return read_len;
        }
        if (static_cast<size_t>(read_len) > len) {
          return ssize_t(-1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      return read_len;
    });

  std::future_status status = future.wait_for(timeout);
  end_flag.store(true, std::memory_order_relaxed);
  if (status == std::future_status::ready) {
    return future.get();
  } else {
    return ssize_t(0);
  }
}

void Serial::flush()
{
  tcflush(fd, TCIFLUSH);
  tcflush(fd, TCOFLUSH);
}

}  // namespace serial
