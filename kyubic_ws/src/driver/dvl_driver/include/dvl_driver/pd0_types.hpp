/**
 * @file pd0_types.hpp
 * @brief Teledyne RDI PD0 Data Format
 * @author R.Ohnishi
 * @date 2025/12/15
 *
 * @details DVL(Path Finder) のPD0データ構造を定義
 ****************************************************/

#ifndef _PD0_TYPES_HPP
#define _PD0_TYPES_HPP

#include <cstdint>
#include <vector>

namespace driver::dvl_driver::path_finder::pd0
{

// Header ID
const uint16_t DATA_ID = 0x7F7F;

struct FixedLeader
{
  uint8_t cpu_fw_ver;
  uint8_t cpu_fw_rev;
  uint16_t system_config;
  uint8_t sim_flag;
  uint8_t lag_length;
  uint8_t num_beams;
  uint8_t num_cells;  // Number of Depth Cells
  uint16_t pings_per_ensemble;
  uint16_t depth_cell_length;     // cm
  uint16_t blank_after_transmit;  // cm
  uint8_t profiling_mode;
  uint8_t low_corr_thresh;
  uint8_t no_code_rep;
  uint8_t percent_good_min;
  uint16_t error_vel_max;  // mm/s
  uint8_t tpp_minutes;
  uint8_t tpp_seconds;
  uint8_t tpp_hundredths;
  uint8_t coord_transform;
  uint16_t heading_alignment;  // 0.01 deg
  uint16_t heading_bias;       // 0.01 deg
  uint8_t sensor_source;
  uint8_t sensors_available;
  uint16_t bin1_distance;      // cm
  uint16_t xmit_pulse_length;  // cm
  uint16_t wp_ref_layer_avg;
  uint8_t false_target_thresh;
  uint16_t transmit_lag_distance;
  uint16_t system_bandwidth;
  uint32_t serial_number;
};

struct VariableLeader
{
  uint16_t ensemble_number;
  uint8_t rtc_year;
  uint8_t rtc_month;
  uint8_t rtc_day;
  uint8_t rtc_hour;
  uint8_t rtc_minute;
  uint8_t rtc_second;
  uint8_t rtc_hundredths;
  uint16_t ensemble_rollovers;
  uint16_t bit_result;
  uint16_t speed_of_sound;    // m/s
  uint16_t transducer_depth;  // dm
  uint16_t heading;           // 0.01 deg
  int16_t pitch;              // 0.01 deg
  int16_t roll;               // 0.01 deg
  uint16_t salinity;          // ppt
  int16_t temperature;        // 0.01 deg C
  uint8_t mpt_minutes;
  uint8_t mpt_seconds;
  uint8_t mpt_hundredths;
  uint16_t heading_std_dev;
  uint16_t pitch_std_dev;
  uint16_t roll_std_dev;
  uint16_t adc_channels[8];    // Converted to meaningful uint16 where applicable
  uint32_t pressure;           // daPa
  uint32_t pressure_variance;  // daPa
  uint8_t leak_status;
  uint16_t leak_a_count;
  uint16_t leak_b_count;
  uint16_t tx_voltage;            // 0.001 V
  uint16_t tx_current;            // 0.001 A
  uint16_t transducer_impedance;  // 0.001 Ohm
};

struct BottomTrack
{
  uint16_t pings_per_ensemble;
  uint8_t corr_mag_min;
  uint8_t eval_amp_min;
  uint8_t mode;
  uint16_t error_vel_max;
  uint32_t range[4];    // cm (Combined LSB/MSB)
  int16_t velocity[4];  // mm/s
  uint8_t correlation[4];
  uint8_t eval_amp[4];
  uint8_t percent_good[4];
  uint16_t ref_layer_min;
  uint16_t ref_layer_near;
  uint16_t ref_layer_far;
  int16_t ref_velocity[4];
  uint8_t ref_correlation[4];
  uint8_t ref_echo_intensity[4];
  uint8_t ref_percent_good[4];
  uint16_t max_depth;  // dm
  uint8_t rssi_amp[4];
  uint8_t gain;
};

struct Pd0Ensemble
{
  bool is_valid = false;
  FixedLeader fixed_leader;
  VariableLeader variable_leader;
  BottomTrack bottom_track;

  // Arrays for per-cell data. Size will be num_cells.
  // Inner vector/array size is 4 (beams).
  bool has_velocity = false;
  std::vector<std::vector<int16_t>> velocity;  // mm/s [cell][beam]

  bool has_correlation = false;
  std::vector<std::vector<uint8_t>> correlation;  // [cell][beam] (0-255)

  bool has_echo_intensity = false;
  std::vector<std::vector<uint8_t>> echo_intensity;  // [cell][beam] (approx 0.45 dB/count)

  bool has_percent_good = false;
  std::vector<std::vector<uint8_t>> percent_good;  // [cell][beam] (0-100)

  bool has_status = false;
  std::vector<std::vector<uint8_t>> status;  // [cell][beam] (0=Bad, 1=Good?)
};

}  // namespace driver::dvl_driver::path_finder::pd0
#endif
