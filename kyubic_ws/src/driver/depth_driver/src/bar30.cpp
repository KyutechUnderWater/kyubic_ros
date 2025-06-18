/**
 * @file bar30.cpp
 * @brief bar30 library
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details bar30のデータを取得
 **************************************************/

#include "depth_driver/bar30.hpp"

#include <cstring>

using namespace std::chrono_literals;

namespace depth_driver
{

Bar30::Bar30(const char * _portname, const int _baudrate) : portname(_portname), baudrate(_baudrate)
{
  serial_ = std::make_shared<serial::Serial>(portname, baudrate);
}

bool Bar30::update()
{
  uint8_t buf[255];
  uint8_t buf_bytes[12];
  serial_->flush();

  serial_->read_until(buf, 255, '#', 100ms);
  ssize_t len = serial_->read_until(buf, sizeof(DepthData) * 2, '%', 100ms);

  if (len > 0) {
    for (size_t i = 0; i < sizeof(DepthData); i++) {
      // 2文字を string にしてから 16進数 → 10進数へ変換
      std::string hex_byte;
      hex_byte += buf[i * 2];
      hex_byte += buf[i * 2 + 1];
      buf_bytes[i] = static_cast<uint8_t>(std::stoi(hex_byte, nullptr, 16));
    }
    data = std::bit_cast<DepthData>(buf_bytes);
    return true;
  }
  return false;
}

DepthData Bar30::get_data() { return data; }

}  // namespace depth_driver
