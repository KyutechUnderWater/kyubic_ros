/**
 * @file pd0_parser.cpp
 * @brief Parser for Teledyne RDI PD0 Data Format
 * @author R.Ohnishi
 * @date 2025/12/15
 *
 * @details DVL(Path Finder) のPD0データをデコードする
 ****************************************************/

#include "dvl_driver/pd0_parser.hpp"

#include "dvl_driver/endian_utils.hpp"

namespace dvl_driver::path_finder::pd0
{
using namespace dvl_driver::utils;

Pd0Parser::Pd0Parser() {}
Pd0Parser::~Pd0Parser() {}

bool Pd0Parser::parse(const unsigned char * buffer, size_t length, Pd0Ensemble & out_data)
{
  out_data = Pd0Ensemble();  // Clear

  if (length < 6) return false;
  if (get_u16(buffer) != DATA_ID) return false;  // Header check

  uint16_t ensemble_len = get_u16(buffer + 2);
  if (length < ensemble_len) return false;  // Incomplete

  uint8_t num_data_types = get_u8(buffer + 5);
  size_t offset_base = 6;

  for (int i = 0; i < num_data_types; ++i) {
    uint16_t offset = get_u16(buffer + offset_base + 2 * i);
    if (offset == 0 || offset >= ensemble_len) continue;

    uint16_t id = get_u16(buffer + offset);

    switch (id) {
      case 0x0000:  // Fixed Leader
      {
        const unsigned char * ptr = buffer + offset;
        out_data.fixed_leader.cpu_fw_ver = get_u8(ptr + 2);
        out_data.fixed_leader.cpu_fw_rev = get_u8(ptr + 3);
        out_data.fixed_leader.system_config = get_u16(ptr + 4);
        out_data.fixed_leader.sim_flag = get_u8(ptr + 6);
        out_data.fixed_leader.lag_length = get_u8(ptr + 7);
        out_data.fixed_leader.num_beams = get_u8(ptr + 8);
        out_data.fixed_leader.num_cells = get_u8(ptr + 9);
        out_data.fixed_leader.pings_per_ensemble = get_u16(ptr + 10);
        out_data.fixed_leader.depth_cell_length = get_u16(ptr + 12);
        out_data.fixed_leader.blank_after_transmit = get_u16(ptr + 14);
        out_data.fixed_leader.profiling_mode = get_u8(ptr + 16);
        out_data.fixed_leader.low_corr_thresh = get_u8(ptr + 17);
        out_data.fixed_leader.no_code_rep = get_u8(ptr + 18);
        out_data.fixed_leader.percent_good_min = get_u8(ptr + 19);
        out_data.fixed_leader.error_vel_max = get_u16(ptr + 20);
        out_data.fixed_leader.tpp_minutes = get_u8(ptr + 22);
        out_data.fixed_leader.tpp_seconds = get_u8(ptr + 23);
        out_data.fixed_leader.tpp_hundredths = get_u8(ptr + 24);
        out_data.fixed_leader.coord_transform = get_u8(ptr + 25);
        out_data.fixed_leader.heading_alignment = get_u16(ptr + 26);
        out_data.fixed_leader.heading_bias = get_u16(ptr + 28);
        out_data.fixed_leader.sensor_source = get_u8(ptr + 30);
        out_data.fixed_leader.sensors_available = get_u8(ptr + 31);
        out_data.fixed_leader.bin1_distance = get_u16(ptr + 32);
        out_data.fixed_leader.xmit_pulse_length = get_u16(ptr + 34);
        out_data.fixed_leader.false_target_thresh = get_u8(ptr + 38);
        out_data.fixed_leader.transmit_lag_distance = get_u16(ptr + 40);
        out_data.fixed_leader.system_bandwidth = get_u16(ptr + 50);
        out_data.fixed_leader.serial_number = get_u32(ptr + 54);
        // Fixed Leader is 58 bytes.
      } break;
      case 0x0080:  // Variable Leader
      {
        const unsigned char * ptr = buffer + offset;
        out_data.variable_leader.ensemble_number = get_u16(ptr + 2);
        out_data.variable_leader.rtc_year = get_u8(ptr + 4);
        out_data.variable_leader.rtc_month = get_u8(ptr + 5);
        out_data.variable_leader.rtc_day = get_u8(ptr + 6);
        out_data.variable_leader.rtc_hour = get_u8(ptr + 7);
        out_data.variable_leader.rtc_minute = get_u8(ptr + 8);
        out_data.variable_leader.rtc_second = get_u8(ptr + 9);
        out_data.variable_leader.rtc_hundredths = get_u8(ptr + 10);
        out_data.variable_leader.ensemble_rollovers = get_u8(ptr + 11);
        out_data.variable_leader.bit_result = get_u16(ptr + 12);
        out_data.variable_leader.speed_of_sound = get_u16(ptr + 14);
        out_data.variable_leader.transducer_depth = get_u16(ptr + 16);
        out_data.variable_leader.heading = get_u16(ptr + 18);
        out_data.variable_leader.pitch = get_s16(ptr + 20);
        out_data.variable_leader.roll = get_s16(ptr + 22);
        out_data.variable_leader.salinity = get_u16(ptr + 24);
        out_data.variable_leader.temperature = get_s16(ptr + 26);
        out_data.variable_leader.mpt_minutes = get_u8(ptr + 28);
        out_data.variable_leader.mpt_seconds = get_u8(ptr + 29);
        out_data.variable_leader.mpt_hundredths = get_u8(ptr + 30);
        out_data.variable_leader.heading_std_dev = get_u8(ptr + 31);
        out_data.variable_leader.pitch_std_dev = get_u8(ptr + 32);
        out_data.variable_leader.roll_std_dev = get_u8(ptr + 33);
        for (int k = 0; k < 8; ++k) {
          out_data.variable_leader.adc_channels[k] = get_u8(ptr + 34 + k);
        }
        out_data.variable_leader.pressure = get_u32(ptr + 48);
        out_data.variable_leader.pressure_variance = get_u32(ptr + 52);
        out_data.variable_leader.leak_status = get_u8(ptr + 66);
        out_data.variable_leader.leak_a_count = get_u16(ptr + 67);
        out_data.variable_leader.leak_b_count = get_u16(ptr + 69);
        out_data.variable_leader.tx_voltage = get_u16(ptr + 71);
        out_data.variable_leader.tx_current = get_u16(ptr + 73);
        out_data.variable_leader.transducer_impedance = get_u16(ptr + 75);
        // Variable Leader is 77 bytes.
      } break;
      case 0x0100:  // Velocity
      {
        out_data.has_velocity = true;
        int num_cells = out_data.fixed_leader.num_cells;
        const unsigned char * ptr = buffer + offset + 2;
        out_data.velocity.resize(num_cells);
        for (int c = 0; c < num_cells; ++c) {
          out_data.velocity[c].resize(4);
          for (int b = 0; b < 4; ++b) {
            out_data.velocity[c][b] = get_s16(ptr + (c * 4 + b) * 2);
          }
        }
      } break;
      case 0x0200:  // Correlation Magnitude
      {
        out_data.has_correlation = true;
        int num_cells = out_data.fixed_leader.num_cells;
        int num_beams = out_data.fixed_leader.num_beams;
        const unsigned char * ptr = buffer + offset + 2;

        out_data.correlation.resize(num_cells);
        for (int c = 0; c < num_cells; ++c) {
          out_data.correlation[c].resize(num_beams);
          for (int b = 0; b < num_beams; ++b) {
            out_data.correlation[c][b] = get_u8(ptr + (c * num_beams + b));
          }
        }
      } break;

      case 0x0300:  // Echo Intensity
      {
        out_data.has_echo_intensity = true;
        int num_cells = out_data.fixed_leader.num_cells;
        int num_beams = out_data.fixed_leader.num_beams;
        const unsigned char * ptr = buffer + offset + 2;

        out_data.echo_intensity.resize(num_cells);
        for (int c = 0; c < num_cells; ++c) {
          out_data.echo_intensity[c].resize(num_beams);
          for (int b = 0; b < num_beams; ++b) {
            out_data.echo_intensity[c][b] = get_u8(ptr + (c * num_beams + b));
          }
        }
      } break;

      case 0x0400:  // Percent Good
      {
        out_data.has_percent_good = true;
        int num_cells = out_data.fixed_leader.num_cells;
        int num_beams = out_data.fixed_leader.num_beams;
        const unsigned char * ptr = buffer + offset + 2;

        out_data.percent_good.resize(num_cells);
        for (int c = 0; c < num_cells; ++c) {
          out_data.percent_good[c].resize(num_beams);
          for (int b = 0; b < num_beams; ++b) {
            out_data.percent_good[c][b] = get_u8(ptr + (c * num_beams + b));
          }
        }
      } break;

      case 0x0500:  // Status Data
      {
        out_data.has_status = true;
        int num_cells = out_data.fixed_leader.num_cells;
        int num_beams = out_data.fixed_leader.num_beams;
        const unsigned char * ptr = buffer + offset + 2;

        out_data.status.resize(num_cells);
        for (int c = 0; c < num_cells; ++c) {
          out_data.status[c].resize(num_beams);
          for (int b = 0; b < num_beams; ++b) {
            out_data.status[c][b] = get_u8(ptr + (c * num_beams + b));
          }
        }
      } break;
      case 0x0600:  // Bottom Track
      {
        const unsigned char * ptr = buffer + offset;
        out_data.bottom_track.pings_per_ensemble = get_u16(ptr + 2);
        out_data.bottom_track.corr_mag_min = get_u8(ptr + 6);
        out_data.bottom_track.eval_amp_min = get_u8(ptr + 7);
        out_data.bottom_track.mode = get_u8(ptr + 9);
        out_data.bottom_track.error_vel_max = get_u16(ptr + 10);

        for (int b = 0; b < 4; ++b) {
          out_data.bottom_track.range[b] = get_u16(ptr + 16 + b * 2);
          out_data.bottom_track.velocity[b] = get_s16(ptr + 24 + b * 2);
          out_data.bottom_track.correlation[b] = get_u8(ptr + 32 + b);
          out_data.bottom_track.eval_amp[b] = get_u8(ptr + 36 + b);
          out_data.bottom_track.percent_good[b] = get_u8(ptr + 40 + b);
        }

        out_data.bottom_track.ref_layer_min = get_u16(ptr + 44);
        out_data.bottom_track.ref_layer_near = get_u16(ptr + 46);
        out_data.bottom_track.ref_layer_far = get_u16(ptr + 48);
        for (int b = 0; b < 4; ++b) {
          out_data.bottom_track.ref_velocity[b] = get_s16(ptr + 50 + b * 2);
          out_data.bottom_track.ref_correlation[b] = get_u8(ptr + 58 + b);
          out_data.bottom_track.ref_echo_intensity[b] = get_u8(ptr + 62 + b);
          out_data.bottom_track.ref_percent_good[b] = get_u8(ptr + 66 + b);
        }
        out_data.bottom_track.max_depth = get_u16(ptr + 70);
        for (int b = 0; b < 4; ++b) out_data.bottom_track.rssi_amp[b] = get_u8(ptr + 72 + b);
        out_data.bottom_track.gain = get_u8(ptr + 76);

        // Range MSB handling
        for (int b = 0; b < 4; ++b) {
          uint8_t msb = get_u8(ptr + 77 + b);
          out_data.bottom_track.range[b] |= (static_cast<uint32_t>(msb) << 16);
        }
        // Bottom Track Leader is 81 bytes.
      } break;
    }
  }

  out_data.is_valid = true;
  return true;
}

}  // namespace dvl_driver::path_finder::pd0
