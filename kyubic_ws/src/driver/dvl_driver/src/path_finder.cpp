/**
 * @file path_finder.cpp
 * @brief Path Finder library
 * @author R.Ohnishi
 * @date 2024/10/30
 *
 * @details DVL(Path Finder) のデータを取得
 * Reference: YusukeMizoguchi on 2022/05/07.
 **********************************************/

#include "dvl_driver/path_finder.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <ostream>
#include <thread>

namespace dvl_driver::path_finder
{

void error(const char * msg)
{
  perror(msg);
  exit(0);
}

// TODO: socket通信のラッパーライブラリを作る
// TODO: socketのcloseをデコンストラクタに入れる
Listener::Listener(const char * _address, const int _port, const int _timeout)
: address(_address), port(_port), timeout(_timeout)
{
  // AF_INET: IPv4, SOCK_STREAM: TCP
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    error("ERROR opening socket");
  }

  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr(address);
  server.sin_port = htons(port);

  if (connect(sockfd, (struct sockaddr *)&server, sizeof(server)) < 0) {
    error("DVL Listener: ERROR connecting");
  }

  // Set time out
  struct timeval tv;
  tv.tv_sec = static_cast<int>(timeout / 1000);
  tv.tv_usec = (timeout - tv.tv_sec * 1000) * 1000;
  std::cout << tv.tv_sec << " " << tv.tv_usec << std::endl;
  if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    std::cout << "ERROR setting socket timeout" << std::endl;
  }

  std::cout << "DVL Listener Start!!!" << std::endl;
}

ssize_t Listener::read(unsigned char * buffer, size_t size) { return ::read(sockfd, buffer, size); }

bool Listener::listen()
{
  memset(buffer, 0, sizeof(buffer));
  ssize_t num_of_listen = this->read(buffer, sizeof(buffer));

  if (num_of_listen == 88) {
    return _parse();
  } else if (num_of_listen < 0) {
    std::cout << "Failed to Listen!" << std::endl;
  } else {
    std::cout << "Missing Data" << std::endl;
  }

  return false;
}

bool Listener::_parse()
{
  auto concat_bit_uint16 = [](uint8_t lsb, uint8_t msb) -> uint16_t {
    return static_cast<uint16_t>(msb << 8) | static_cast<uint16_t>(lsb);
  };

  auto concat_bit_int16 = [](uint8_t lsb, uint8_t msb) -> int16_t {
    return static_cast<int16_t>(msb << 8) | static_cast<int16_t>(lsb);
  };

  for (auto buf : buffer) {
    bit_array.emplace_back(buf);
  }

  dvl_data.pathfinder_id = buffer[DATA_ID];
  dvl_data.data_structure = buffer[DATA_STRUCTURE];
  dvl_data.num_of_bytes = buffer[NUM_OF_BYTES];
  dvl_data.system_config = buffer[SYSTEM_CONFIG];

  // TODO: bit arrayを使わず，実装
  if (bit_array.at(LEAK_SENSOR).any() || bit_array.at(LEAK_SENSOR + 1).any()) {
    std::cout << "Leaking or Circuit short in DVL！" << std::endl;
    return false;
  }

  dvl_data.tofp_hour = buffer[TOFP_HOUR];
  dvl_data.tofp_minute = buffer[TOFP_MINUTE];
  dvl_data.tofp_seconds = buffer[TOFP_SECOND];
  dvl_data.tofp_hundreds = buffer[TOFP_HUNDREDTHS];

  // speed of sound
  dvl_data.speed_of_sound = concat_bit_uint16(buffer[SPEED_OF_SOUND], buffer[SPEED_OF_SOUND + 1]);

  // temperature
  dvl_data.temperature = concat_bit_uint16(buffer[TEMPERATURE], buffer[TEMPERATURE + 1]);

  // velocity
  dvl_data.x_vel_bottom =
    (concat_bit_int16(buffer[X_VEL_BTM], buffer[X_VEL_BTM + 1]) * cos(3 * M_PI_4) -
     concat_bit_int16(buffer[Y_VEL_BTM], buffer[Y_VEL_BTM + 1]) * sin(3 * M_PI_4)) *
    0.001;
  dvl_data.y_vel_bottom =
    (concat_bit_int16(buffer[X_VEL_BTM], buffer[X_VEL_BTM + 1]) * sin(3 * M_PI_4) +
     concat_bit_int16(buffer[Y_VEL_BTM], buffer[Y_VEL_BTM + 1]) * cos(3 * M_PI_4)) *
    (-0.001);
  dvl_data.z_vel_bottom = concat_bit_int16(buffer[Z_VEL_BTM], buffer[Z_VEL_BTM + 1]) * 0.001;
  dvl_data.e_vel_bottom = concat_bit_int16(buffer[E_VEL_BTM], buffer[E_VEL_BTM + 1]);
  if (dvl_data.e_vel_bottom == -32768) {
    dvl_data.x_vel_bottom = dvl_data.y_vel_bottom = dvl_data.z_vel_bottom = 0.0;
  }

  // rotate
  dvl_data.pitch = concat_bit_int16(buffer[PITCH], buffer[PITCH + 1]) * 0.01;
  dvl_data.roll = concat_bit_int16(buffer[ROLL], buffer[ROLL + 1]) * 0.01;
  dvl_data.heading = concat_bit_int16(buffer[HEADING], buffer[HEADING + 1]) * 0.01;

  // altitude
  dvl_data.range_to_bottom_bm1 =
    concat_bit_int16(buffer[BM1_RNG_TO_BTM], buffer[BM1_RNG_TO_BTM + 1]) * 0.01;
  dvl_data.range_to_bottom_bm2 =
    concat_bit_int16(buffer[BM2_RNG_TO_BTM], buffer[BM2_RNG_TO_BTM + 1]) * 0.01;
  dvl_data.range_to_bottom_bm3 =
    concat_bit_int16(buffer[BM3_RNG_TO_BTM], buffer[BM3_RNG_TO_BTM + 1]) * 0.01;
  dvl_data.range_to_bottom_bm4 =
    concat_bit_int16(buffer[BM4_RNG_TO_BTM], buffer[BM4_RNG_TO_BTM + 1]) * 0.01;
  dvl_data.altitude = (dvl_data.range_to_bottom_bm1 + dvl_data.range_to_bottom_bm2 +
                       dvl_data.range_to_bottom_bm3 + dvl_data.range_to_bottom_bm4) /
                      4;
  // TODO:num of good beamを考慮してaltitudeを求める

  return true;
}

std::shared_ptr<Data> Listener::get_dvl_data() { return std::make_shared<Data>(dvl_data); }

void Listener::print_info()
{
  // TODO:Diagnotics化
  if (dvl_data.pathfinder_id == 125) {
    std::cout << "Path Finder ID is correct" << std::endl;
  } else {
    std::cout << "This Packet is not from Path Finder" << std::endl;
  }

  std::cout << "Setting of Speed of Sound is " << dvl_data.speed_of_sound << std::endl;
  std::cout << "TOFP Hour:Minutes:Second:Hundredth: " << dvl_data.tofp_hour << " : "
            << dvl_data.tofp_minute << " : " << dvl_data.tofp_seconds << std::endl;
  std::cout << "Temperature is " << dvl_data.temperature << "[°C]" << std::endl;
  std::cout << "X-VEL-BTM is " << dvl_data.x_vel_bottom << "[m/s]" << std::endl;
  std::cout << "Y-VEL-BTM is " << dvl_data.y_vel_bottom << "[m/s]" << std::endl;
  std::cout << "Z-VEL-BTM is " << dvl_data.z_vel_bottom << "[m/s]" << std::endl;

  std::cout << "E-VEL-BTM is " << dvl_data.e_vel_bottom << "[m/s]" << std::endl;

  std::cout << "pitch is " << dvl_data.pitch << "[deg]" << std::endl;
  std::cout << "roll is " << dvl_data.roll << "[deg]" << std::endl;
  std::cout << "heading is " << dvl_data.heading << "[deg]" << std::endl;
  std::cout << "range_to_bottom_bm1 is " << dvl_data.range_to_bottom_bm1 << "[m]" << std::endl;
  std::cout << "range_to_bottom_bm2 is " << dvl_data.range_to_bottom_bm2 << "[m]" << std::endl;
  std::cout << "range_to_bottom_bm3 is " << dvl_data.range_to_bottom_bm3 << "[m]" << std::endl;
  std::cout << "range_to_bottom_bm4 is " << dvl_data.range_to_bottom_bm4 << "[m]" << std::endl;
  std::cout << "Altitude is " << dvl_data.altitude << "[m]" << std::endl;
  bit_array.clear();
  std::cout << "bit_array is cleard  " << std::endl;
}

Sender::Sender(const char * _address, const int _port, const int _timeout)
: address(_address), port(_port), timeout(_timeout)
{
  // AF_INET: IPv4, SOCK_STREAM: TCP
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    error("ERROR opening socket");
  }

  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr(address);
  server.sin_port = htons(port);

  if (connect(sockfd, (struct sockaddr *)&server, sizeof(server)) < 0) {
    error("DVL Sender: ERROR connecting");
  }

  // Set time out
  struct timeval tv;
  tv.tv_sec = static_cast<int>(timeout / 1000);
  tv.tv_usec = (timeout - tv.tv_sec * 1000) * 1000;
  if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    error("ERROR setting socket timeout");
  }

  std::cout << "DVL Sender Start!!!" << std::endl;
}

ssize_t Sender::read(unsigned char * buffer, size_t size) { return ::read(sockfd, buffer, size); }

bool Sender::send_cmd(const std::string & cmd, const uint & wait_time)
{
  if (send(sockfd, cmd.c_str(), cmd.size(), 0) != (ssize_t)cmd.size()) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::seconds(wait_time));
  return true;
}

bool Sender::send_break_cmd() { return send_cmd(break_cmd, 4); }

bool Sender::send_ping_cmd() { return send_cmd(ping_cmd); }

}  // namespace dvl_driver::path_finder
