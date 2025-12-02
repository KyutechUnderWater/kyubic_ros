/**
 * @file path_finder.hpp
 * @brief Path Finder library
 * @author R.Ohnishi
 * @date 2024/10/30
 *
 * @details DVL(Path Finder) のデータを取得
 * Reference: YusukeMizoguchi on 2022/05/07.
 **********************************************/

#ifndef _PATH_FINDER_HPP
#define _PATH_FINDER_HPP

#include <netinet/in.h>

#include <bitset>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/**
 * @namespace dvl_driver
 * @brief For dvl driver
 */
namespace dvl_driver::path_finder
{

/**
 * @brief Table of data format
 *
 * @details Bit position of data.
 * DVLのreferenceのバイト番号-1
 */
enum DATA_TABLE
{
  DATA_ID = 0,
  DATA_STRUCTURE = 1,
  NUM_OF_BYTES = 2,
  SYSTEM_CONFIG = 4,
  X_VEL_BTM = 5,
  Y_VEL_BTM = 7,
  Z_VEL_BTM = 9,
  E_VEL_BTM = 11,
  BM1_RNG_TO_BTM = 13,
  BM2_RNG_TO_BTM = 15,
  BM3_RNG_TO_BTM = 17,
  BM4_RNG_TO_BTM = 19,
  BOTTOM_STATUS = 21,
  X_VEL_REF_LAYER = 22,
  Y_VEL_REF_LAYER = 24,
  Z_VEL_REF_LAYER = 26,
  E_VEL_REF_LAYER = 28,
  REF_LAYER_START = 30,
  REF_LAYER_END = 32,
  REF_LAYER_STATUS = 34,
  TOFP_HOUR = 35,
  TOFP_MINUTE = 36,
  TOFP_SECOND = 37,
  TOFP_HUNDREDTHS = 38,
  LEAK_SENSOR = 39,
  SPEED_OF_SOUND = 41,
  TEMPERATURE = 43,
  SALINITY = 45,
  DEPTH = 46,
  PITCH = 48,
  ROLL = 50,
  HEADING = 52,
  CALED_DISTANCE_TO_BOTTOM_EAST = 54,
  CALED_DISTANCE_TO_BOTTOM_NORTH = 58,
  CALED_DISTANCE_TO_BOTTOM_UP = 62,
  CALED_DISTANCE_TO_ERROR = 66,
  CALED_DISTANCE_TO_REF_EAST = 70,
  CALED_DISTANCE_TO_REF_NORTH = 74,
  CALED_DISTANCE_TO_REF_UP = 78,
  CALED_DISTANCE_TO_REF_ERROR = 82,
  CHECKSUM = 86
};

/**
 * @brief Store data with modified type of Path Finder data
 */
struct Data  // PD5
{
  unsigned char pathfinder_id;
  std::string data_structure;
  bool DVL_VEL_ERROR;
  uint8_t num_of_bytes;
  uint8_t system_config;
  float x_vel_bottom;
  float y_vel_bottom;
  float z_vel_bottom;
  int16_t e_vel_bottom;
  float range_to_bottom_bm1;
  float range_to_bottom_bm2;
  float range_to_bottom_bm3;
  float range_to_bottom_bm4;
  unsigned char bottom_status;
  float altitude;
  float velocity[4];
  float ref_layer_start;
  float ref_layer_end;
  unsigned char ref_layer_status;
  uint16_t tofp_hour;
  uint16_t tofp_minute;
  uint16_t tofp_seconds;
  uint16_t tofp_hundreds;
  uint8_t leak_status;
  uint16_t speed_of_sound;
  float temperature;
  uint8_t salinity;
  uint8_t depth;
  float pitch;    //[degree]
  float roll;     //[degree]
  float heading;  //[degree]
  float dmg_distance_bottom_east;
  float dmg_distance_bottom_north;
  float dmg_distance_bottom_up;
  float dmg_distance_ref_east;
  float dmg_distance_ref_north;
  float dmg_distance_ref_up;
  float dmg_distance_ref_error;
  bool BEAM_4_LOW_ECHO;
  bool BEAM_4_LOW_CORRELATION;
  bool BEAM_3_LOW_ECHO;
  bool BEAM_3_LOW_CORRELATION;
  bool BEAM_2_LOW_ECHO;
  bool BEAM_2_LOW_CORRELATION;
  bool BEAM_1_LOW_ECHO;
  bool BEAM_1_LOW_CORRELATION;
};

/**
 * @brief Listener class
 */
class Listener
{
private:
  const char * address;
  const int port;
  const int timeout_ms;

  int sockfd;
  struct sockaddr_in server;

  unsigned char buffer[88];
  std::vector<std::bitset<8>> bit_array;
  Data dvl_data;

  /**
   * @brief parse binary data listen from Path Finder
   * @return True if successful, False otherwise
   */
  bool _parse();

  /**
   * @brief Concatenate two 8-bit data into uint16
   * @return data of uint16 type
   */
  uint16_t concat_bit_uint16(uint8_t lsb, uint8_t msb);

  /**
   * @brief Concatenate two 8-bit data into int16
   * @return data of int16 type
   */
  int16_t concat_bit_int16(uint8_t, uint8_t);

  // char topic_message[256] = {0}; // maybeunused
  // int num_of_listen, choice = 1;
  // kyubic_interface_msgs::msg::PathFinder dvl_msg;
  // kyubic_interface_msgs::PathFinderAux dvl_auxmsg;

public:
  /**
   * @brief Create socket and Connect Socket
   * @param address Ip address of Path Finder
   * @param port Port of Path Finder
   * @param timeout_ms socket timeout [msec]
   * @details Socket creation with IPv4 and TCP. Socket connection based on address and port
   */
  explicit Listener(const char * _address, const int _port, const int _timeout_ms = 1000);

  /**
   * @brief Listen and parse data from Path Finder
   * @return True if successful, False otherwise
   */
  bool listen();

  /**
   * @brief read data from Path Finder
   * @return Number of bytes of data received
   */
  ssize_t read(unsigned char * buffer, size_t size);

  /**
   * @brief Get various Path Finder data listend
   * @return various dvl data of DVLData type
   */
  std::shared_ptr<Data> get_dvl_data();

  /**
   * @brief Print various Path Finder data listend
   */
  void print_info();
};

class Sender
{
  const char * address;
  const int port;
  const int timeout_ms;

  int sockfd;
  struct sockaddr_in server;

  const std::string ping_cmd = "CS\r\n";
  const std::string break_cmd = "===";

public:
  /**
   * @brief Create socket and Connect Socket
   * @param address Ip address of Path Finder
   * @param port Port of Path Finder
   * @param timeout_ms socket timeout [msec]
   * @details Socket creation with IPv4 and TCP. Socket connection based on address and port
   */
  explicit Sender(const char * _address, const int _port, const int _timeout_ms = 1000);

  /**
   * @brief read data from Path Finder
   * @return Number of bytes of data received
   */
  ssize_t read(unsigned char * buffer, size_t size);

  /**
   * @brief Send command to Path Finder
   */
  bool send_cmd(const std::string & cmd, const uint & wait_time = 0);

  /**
   * @brief Send break command to Path Finder
   */
  bool send_break_cmd();

  /**
   * @brief Send ping command to Path Finder
   */
  bool send_ping_cmd();
};

}  // namespace dvl_driver::path_finder

#endif
