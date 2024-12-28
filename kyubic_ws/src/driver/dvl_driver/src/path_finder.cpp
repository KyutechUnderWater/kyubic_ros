/**
 * @file path_finder.cpp
 * @brief path finder library
 * @author R.Ohnishi
 * @date 2024/10/31
 *
 * @details Path Finderのデータを取得
 * Original: Created by YusukeMizoguchi on 2022/05/07.
 **************************************************/

#include "dvl_driver/path_finder.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>

namespace dvl_driver
{

void error(const char * msg)
{
  perror(msg);
  exit(0);
}

// ip: 172.30.51.100
// port: 1033
DVLListener::DVLListener(const char * _address, const int _port) : address(_address), port(_port)
{
  // AF_INET: IPv4, SOCK_STREAM: TCP
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) error("ERROR opening socket");

  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr(address);
  server.sin_port = htons(port);

  if (connect(sockfd, (struct sockaddr *)&server, sizeof(server)) < 0)
    error("DVLListener: ERROR connecting");

  std::cout << "DVLListener Start!!!" << std::endl;
}

void DVLListener::_parse()
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

  if (bit_array.at(LEAK_SENSOR).any() || bit_array.at(LEAK_SENSOR + 1).any()) {
    error("Leaking or Circuit short in DVL！");
    exit(-1);
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
    (concat_bit_int16(buffer[X_VEL_BTM], buffer[X_VEL_BTM + 1]) * cos(M_PI_4) -
     concat_bit_int16(buffer[Y_VEL_BTM], buffer[Y_VEL_BTM + 1]) * sin(M_PI_4)) *
    0.001;
  dvl_data.y_vel_bottom =
    (concat_bit_int16(buffer[X_VEL_BTM], buffer[X_VEL_BTM + 1]) * sin(M_PI_4) +
     concat_bit_int16(buffer[Y_VEL_BTM], buffer[Y_VEL_BTM + 1]) * cos(M_PI_4)) *
    (-0.001);
  dvl_data.z_vel_bottom = concat_bit_int16(buffer[Z_VEL_BTM], buffer[Z_VEL_BTM + 1]) * 0.001;
  dvl_data.e_vel_bottom = concat_bit_int16(buffer[E_VEL_BTM], buffer[E_VEL_BTM + 1]);
  if (dvl_data.e_vel_bottom == -32768)
    dvl_data.x_vel_bottom = dvl_data.y_vel_bottom = dvl_data.z_vel_bottom = 0.0;

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
}

void DVLListener::listen()
{
  memset(buffer, 0, sizeof(buffer));
  ssize_t num_of_listen = read(sockfd, buffer, sizeof(buffer));

  if (num_of_listen == 88)
    _parse();
  else if (num_of_listen < 0)
    error("Failed to Listen!");
  else
    std::cout << "Missing Data" << std::endl;
}

// kyubic_interface_msgs::msg::PathFinder DVLListener::filldvlmsg(DVL_Data dvl_data) {
//   kyubic_interface_msgs::msg::PathFinder pathfindermsg;
//
//   //  std::cout << duration.nsec;
//   //  std::cout << duration.sec << " " << duration.nsec / 10e9 << std::endl;
//   pathfindermsg.header.frame_id = "/pathfinder";
//   pathfindermsg.velocity.x = dvl_data.x_vel_bottom;
//   pathfindermsg.velocity.y = dvl_data.y_vel_bottom;
//   pathfindermsg.velocity.z = dvl_data.z_vel_bottom;
//   pathfindermsg.velocity_error = dvl_data.e_vel_bottom;
//
//   pathfindermsg.depth = 0.0;
//
//   //  std::cout << pathfindermsg.depth << std::endl;
//   pathfindermsg.altitude = dvl_data.altitude;
//   last_dvl_comm_time = ros::Time::now();
//   return pathfindermsg;
// }

// kyubic_interface_msgs::PathFinderAux DVLListener::fillDvlAuxMsg(DVL_Data dvl_data) {
//   kyubic_interface_msgs::PathFinderAux auxmsg;
//   auxmsg.header.stamp = ros::Time::now();
//   auxmsg.header.frame_id = "/pathfinder";
//   auxmsg.salinity = dvl_data.salinity;
//   auxmsg.temparature = dvl_data.temparature;
//   auxmsg.salinity = dvl_data.salinity;
//   auxmsg.pitch = dvl_data.pitch;
//   auxmsg.heading = dvl_data.heading;
//   auxmsg.range_to_bottom.at(0) = dvl_data.range_to_bottom_bm1;
//   auxmsg.range_to_bottom.at(1) = dvl_data.range_to_bottom_bm2;
//   auxmsg.range_to_bottom.at(2) = dvl_data.range_to_bottom_bm3;
//   auxmsg.range_to_bottom.at(3) = dvl_data.range_to_bottom_bm4;
//   auxmsg.ref_layer_start = dvl_data.ref_layer_start;
//   auxmsg.ref_layer_end = dvl_data.ref_layer_end;
//   auxmsg.ref_layer_status = dvl_data.ref_layer_status;
//   auxmsg.data_struct = dvl_data.data_structure;
//   auxmsg.system_config = dvl_data.system_config;
//   auxmsg.bottom_status = dvl_data.bottom_status;
//   auxmsg.leak_status = dvl_data.leak_status;
//   return auxmsg;
// }

std::shared_ptr<DVLData> DVLListener::get_dvl_data()
{
  return std::make_shared<DVLData>(dvl_data);
}

void DVLListener::print_info()
{
  // TODO:Diagnotics化
  if (dvl_data.pathfinder_id == 125) {
    std::cout << "Path Finder ID is correct" << std::endl;
  } else {
    error("This Packet is not from Path Finder");
  }
  //    if (dvl_data.data_structure == "1") {
  //        std::cout << "Data Structure is PD5" << std::endl;
  //    } else {
  //        error("Data structure is not PD5 ");
  //    }
  //  if (dvl_data.system_config.to_ulong() == 179) {
  //    std::cout << "System Config is Good" << std::endl;
  //  } else {
  //    error("Please Confirm system config");
  //  }
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

// void DVLListener::callback(const std_msgs::String::ConstPtr &msg) {
//   memset(topic_message, 0, 256);
//   strcpy(topic_message, msg->data.c_str());
//   ROS_INFO("I heard:[%s]", msg->data.c_str());
// }
//
// char *DVLListener::getMessageValue() { return topic_message; }

DVLSender::DVLSender(const char * _address, const int _port) : address(_address), port(_port)
{
  // Sender用のソケットの用意
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    error("DVLSender setting is failed！\r\n Please check address");

  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr(address);
  server.sin_port = htons(port);

  if (connect(sockfd, (struct sockaddr *)&server, sizeof(server)) < 0)
    error("Sender: ERROR connecting");

  std::cout << "DVLSender Start!!!" << std::endl;
}

void DVLSender::ping()
{
  if (send(sockfd, ping_char, sizeof(ping_char), 0) != sizeof(ping_char)) error("don't send");
  std::cout << "ping!!" << std::endl;
}

// void DVLSender::cmdcallback(std_msgs::StringConstPtr msg) {
//   char topic_message[256] = {0};
//   char buffer[300];
//   memset(topic_message, 0, 256);
//   strcpy(topic_message, msg->data.c_str());
//   ROS_INFO("I heard:[%s]", msg->data.c_str());
//   write(sockfd, topic_message, strlen(topic_message));
//   std::cout << "dvl responce is " << std::endl;
//   std::cout << read(sockfd, buffer, 300);
// }

// std::string to_binString(unsigned int val) {
//   if (!val)
//     return std::string("0");
//   std::string str;
//   while (val != 0) {
//     if ((val & 1) == 0)             // val は偶数か？
//       str.insert(str.begin(), '0'); //  偶数の場合
//     else
//       str.insert(str.begin(), '1'); //  奇数の場合
//     val >>= 1;
//   }
//   return str;
// }

// void DVLDriver::refLayerStatus(std::bitset<16> layer_status)
//{
//   if (layer_status[3]) {
//     ROS_WARN("BEAM 4 LOW CORRELATION");
//   }
//   if (layer_status[2]) {
//     ROS_WARN("BEAM 3 LOW CORRELATION");
//   }
//   if (layer_status[1]) {
//     ROS_WARN("BEAM 2 LOW CORRELATION");
//   }
//   if (layer_status[0]) {
//     ROS_WARN("BEAM 1 LOW CORRELATION");
//   }
//   if (layer_status[4]) {
//     ROS_WARN("ALTITDE is TOO SHALLOW");
//   }
//   if (!layer_status.any()) {
//     ROS_DEBUG("Layer Status is All OK");
//   }
// }

}  // namespace dvl_driver
