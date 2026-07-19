/**
 * @file dvl75_parser.hpp
 * @brief DVL-75 NMEA measurement types and parser declaration.
 */

#ifndef DVL75_DRIVER__DVL75_PARSER_HPP_
#define DVL75_DRIVER__DVL75_PARSER_HPP_

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <variant>

namespace dvl75_driver
{

/**
 * @brief Measurements decoded from one DVL-75 DVEXT sentence.
 */
struct Dvext
{
  bool bottom_lock;                     ///< Bottom-track lock state.
  std::uint8_t data_skips;              ///< Consecutive data-skip count.
  double velocity_up;                   ///< Upward velocity in metres per second.
  double altitude;                      ///< Altitude along the sensor axis in metres.
  double velocity_north;                ///< North velocity in metres per second.
  double velocity_east;                 ///< East velocity in metres per second.
  std::array<bool, 4> beam_lock;        ///< Bottom-track state for each beam.
  std::array<double, 4> beam_velocity;  ///< Velocity along each beam in metres per second.
  std::array<double, 4> beam_range;     ///< Range along each beam in metres.
};

/**
 * @brief Measurements decoded from one DVL-75 DVPDL sentence.
 */
struct Dvpdl
{
  std::uint64_t device_time_us;          ///< DVL device timestamp in microseconds.
  std::uint32_t delta_time_us;           ///< Interval since the previous DVPDL in microseconds.
  std::array<double, 3> angle_delta;     ///< Angular delta in radians.
  std::array<double, 3> position_delta;  ///< Position delta in metres.
  std::uint8_t confidence;               ///< Measurement confidence in the range 0 to 100.
};

/**
 * @brief Report malformed or invalid DVL-75 NMEA data.
 */
class ParseError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/**
 * @brief Parsed DVL-75 measurement, or unrelated text represented by std::monostate.
 */
using Measurement = std::variant<std::monostate, Dvext, Dvpdl>;

/**
 * @brief Parse one DVL-75 NMEA sentence.
 *
 * @param sentence ASCII sentence received from the DVL.
 * @param validate_checksum Whether missing or mismatched NMEA checksums are rejected.
 * @return Parsed DVEXT or DVPDL measurement, or std::monostate for unrelated text.
 * @throws ParseError If a supported sentence is malformed or fails validation.
 */
Measurement parse_sentence(const std::string & sentence, bool validate_checksum);

}  // namespace dvl75_driver

#endif  // DVL75_DRIVER__DVL75_PARSER_HPP_
