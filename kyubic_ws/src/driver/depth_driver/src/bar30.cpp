/**
 * @file bar30.cpp
 * @brief bar30 library
 * @author R.Ohnishi
 * @date 2024/10/27
 *
 * @details bar30のデータを取得
 **************************************************/

#include "depth_driver/bar30.hpp"

using namespace std::chrono_literals;

namespace depth_driver
{

Bar30::Bar30(const char * _portname, const int _baudrate) : portname(_portname), baudrate(_baudrate)
{
  serial_ = std::make_shared<serial::Serial>(portname, baudrate);
}

bool Bar30::update()
{
  uint8_t buf[100] = {};
  serial_->flush();

  // TODO: 開始文字を設定する
  // serial_->read_until(buf, sizeof(buf), 's', 10ms);
  ssize_t len = serial_->read_until(buf, sizeof(buf), '\n', 100ms);
  if (len > 0) {
    // TODO: binaryで送れるようにする。s->fでエラーが起こると異常終了するので
    std::string s_depth = std::string(buf, buf + len).substr(0, 6);
    depth_data = std::stof(s_depth);

    return true;
  }
  return false;
}

float Bar30::get_depth_data()
{
  return depth_data;
}

}  // namespace depth_driver
