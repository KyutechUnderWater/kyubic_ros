/**
 * @file pd5_parser.cpp
 * @brief Parser for Teledyne RDI PD4/PD5 Data Format
 * @author R.Ohnishi
 * @date 2025/12/15
 *
 * @details DVL(Path Finder) のPD4/PD5データをデコードする
 ********************************************************/

#include "dvl_driver/pd5_parser.hpp"

#include "dvl_driver/endian_utils.hpp"

namespace driver::dvl_driver::path_finder::pd5
{
using namespace driver::dvl_driver::utils;

Pd5Parser::Pd5Parser() {}
Pd5Parser::~Pd5Parser() {}

bool Pd5Parser::parse(const unsigned char * buffer, size_t length, Pd5Ensemble & out_data)
{
  out_data = Pd5Ensemble();

  if (length < 47) return false;
  if (get_u8(buffer) != DATA_ID) return false;

  out_data.id = get_u8(buffer);
  out_data.data_structure = get_u8(buffer + 1);  // 0=PD4, 1=PD5
  out_data.num_bytes = get_u16(buffer + 2);

  if (length < static_cast<size_t>(out_data.num_bytes) + 2) return false;

  out_data.system_config = get_u8(buffer + 4);

  for (int i = 0; i < 4; ++i) {
    out_data.btm_velocity[i] = get_s16(buffer + 5 + i * 2);
  }
  for (int i = 0; i < 4; ++i) {
    out_data.btm_range[i] = get_u16(buffer + 13 + i * 2);
  }

  out_data.bottom_status = get_u8(buffer + 21);

  for (int i = 0; i < 4; ++i) {
    out_data.wt_velocity[i] = get_s16(buffer + 22 + i * 2);
  }

  out_data.ref_layer_start = get_u16(buffer + 30);
  out_data.ref_layer_end = get_u16(buffer + 32);
  out_data.ref_layer_status = get_u8(buffer + 34);

  out_data.tofp_hour = get_u8(buffer + 35);
  out_data.tofp_minute = get_u8(buffer + 36);
  out_data.tofp_second = get_u8(buffer + 37);
  out_data.tofp_hundredths = get_u8(buffer + 38);

  out_data.bit_results = get_u16(buffer + 39);
  out_data.speed_of_sound = get_u16(buffer + 41);
  out_data.temperature = get_s16(buffer + 43);

  if (out_data.data_structure == 1 && length >= 88) {  // PD5 extension
    out_data.has_pd5_extension = true;
    out_data.salinity = get_u8(buffer + 45);
    out_data.depth = get_u16(buffer + 46);
    out_data.pitch = get_s16(buffer + 48);
    out_data.roll = get_s16(buffer + 50);
    out_data.heading = get_u16(buffer + 52);

    for (int i = 0; i < 4; ++i) {
      out_data.dmg_btm[i] = get_s32(buffer + 54 + i * 4);
    }
    for (int i = 0; i < 4; ++i) {
      out_data.dmg_ref[i] = get_s32(buffer + 70 + i * 4);
    }
  }

  out_data.is_valid = true;
  return true;
}

}  // namespace driver::dvl_driver::path_finder::pd5
