/**
 * @file pd5_types.hpp
 * @brief Teledyne RDI PD5 Data Format
 * @author R.Ohnishi
 * @date 2025/12/15
 *
 * @details DVL(Path Finder) のPD5データ構造を定義
 ****************************************************/

#ifndef _PD5_TYPES_HPP
#define _PD5_TYPES_HPP

#include <cstdint>

namespace driver::dvl_driver::path_finder::pd5
{

// Header ID
const uint8_t DATA_ID = 0x7D;

struct Pd5Ensemble
{
  bool is_valid = false;

  // Header Info
  uint8_t id;              // 0x7D
  uint8_t data_structure;  // 0=PD4, 1=PD5
  uint16_t num_bytes;      // The number of bytes sent in this data structure

  // System Config
  uint8_t system_config;

  // Bottom Track Velocity (mm/s)
  int16_t btm_velocity[4];  // X, Y, Z, Error

  // Range to Bottom (cm for 300/600kHz)
  uint16_t btm_range[4];

  // Bottom Status
  uint8_t bottom_status;

  // Water Mass Reference Layer Velocity (mm/s)
  int16_t wt_velocity[4];

  // Water Mass Reference Layer Info
  uint16_t ref_layer_start;  // dm
  uint16_t ref_layer_end;    // dm
  uint8_t ref_layer_status;

  // Time of First Ping
  uint8_t tofp_hour;
  uint8_t tofp_minute;
  uint8_t tofp_second;
  uint8_t tofp_hundredths;

  // BIT Results (Status for leak sensors)
  uint16_t bit_results;

  // Environment
  uint16_t speed_of_sound;  // not include (m/s)
  int16_t temperature;      // not include (0.01 deg C)

  // --- PD5 Specific Extension ---
  bool has_pd5_extension = false;
  uint8_t salinity;  // not include (ppt)
  uint16_t depth;    // not include (dm)
  int16_t pitch;     // not include (0.01 deg)
  int16_t roll;      // not include (0.01 deg)
  uint16_t heading;  // not include (0.01 deg)

  // Cumulative distance from the first ping after initialization or <BREAK> reception
  int32_t dmg_btm[4];  // Distance Made Good Bottom (mm)
  int32_t dmg_ref[4];  // Distance Made Good Reference (mm)
};

}  // namespace driver::dvl_driver::path_finder::pd5
#endif
