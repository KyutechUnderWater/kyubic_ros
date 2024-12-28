/**
 * @file path_finder.hpp
 * @brief path finder library
 * @author R.Ohnishi
 * @date 2024/10/31
 *
 * @details Path Finderのデータを取得
 * Original: Created by YusukeMizoguchi on 2022/05/07.
 **************************************************/

#ifndef _PATH_FINDER_HPP
#define _PATH_FINDER_HPP

#include <driver_msgs/msg/path_finder.hpp>
#include <std_msgs/msg/string.hpp>

#include <netinet/in.h>

#include <bitset>
#include <string>

/**
 * @namespace dvl_driver
 * @brief For dvl driver
 */
namespace dvl_driver
{

/**
 * @brief Table of data format
 * @details Number is byte position
 */
enum DVL_DATA_TABLE {
  // DVLのreferenceのバイト番号-1
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
  calcurated_distance_to_EAST_BOTTOM = 54,
  CALED_DISTANCE_TO_NORTH = 58,
  CALED_DISTANCE_TO_UP = 62,
  CALED_DISTANCE_TO_ERROR = 66,
  CALED_DISTANCE_TO_REF = 70,
  CALED_DISTANCE_TO_NORTH_REF = 74,
  CALED_DISTANCE_UP_REF = 78,
  CALED_DISTANCE_ERROR_REF = 82,
  CHECKSUM = 86
};

/**
 * Store dvl data
 */
struct DVLData  // PD5
{
  unsigned char pathfinder_id;
  std::string data_structure;
  bool DVL_VEL_ERROR;
  uint8_t num_of_bytes;
  uint8_t system_config;
  float x_vel_bottom;
  float y_vel_bottom;
  float z_vel_bottom;
  float e_vel_bottom;
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
 * @brief DVLListener class
 */
class DVLListener
{
public:
  /**
   * @brief UDP setting
   */
  explicit DVLListener(const char * _address, const int _port);

  /**
   * @brief
   */
  void listen();

  /**
   * @brief
   */
  std::shared_ptr<DVLData> get_dvl_data();

  /**
   * @brief
   */
  void print_info();

  // kyubic_interface_msgs::msg::PathFinder filldvlmsg(DVL_Data);
  // kyubic_interface_msgs::PathFinderAux fillDvlAuxMsg(DVL_Data);
  // void callback(const std_msgs::msg::String &msg);
  // char *getMessageValue();

private:
  const char * address;
  const int port;

  int sockfd;
  struct sockaddr_in server;

  unsigned char buffer[88];
  std::vector<std::bitset<8>> bit_array;
  DVLData dvl_data;

  void _parse();

  // char topic_message[256] = {0}; // maybeunused
  // int num_of_listen, choice = 1;
  // kyubic_interface_msgs::msg::PathFinder dvl_msg;
  // kyubic_interface_msgs::PathFinderAux dvl_auxmsg;
};

/**
 * @brief DVLSender class
 */
class DVLSender
{
public:
  /**
   * @brief UDP setting
   */
  explicit DVLSender(const char * _address, const int _port);

  /**
   * @brief
   */
  void ping();

  // void cmdcallback(const std_msgs::msg::String);

private:
  const char * address;
  const int port;

  int sockfd;
  struct sockaddr_in server;

  char ping_char[4] = {'C', 'S', '\r', '\n'};
  char break_char[4] = {'='};
  char topic_message[256] = {0};
};

}  // namespace dvl_driver

#endif
