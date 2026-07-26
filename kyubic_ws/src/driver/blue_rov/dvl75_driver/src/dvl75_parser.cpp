/**
 * @file dvl75_parser.cpp
 * @brief Parser implementation for DVL-75 NMEA sentences.
 */

#include "dvl75_driver/dvl75_parser.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <vector>

namespace dvl75_driver
{
namespace
{

std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1U);
}

std::vector<std::string> split(const std::string & value)
{
  std::vector<std::string> fields;
  std::stringstream stream(value);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  if (!value.empty() && value.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}

bool parse_bool(const std::string & value)
{
  if (value == "T") {
    return true;
  }
  if (value == "F") {
    return false;
  }
  throw ParseError("expected T or F, got " + value);
}

double parse_float(const std::string & value, const std::string & name)
{
  try {
    std::size_t position = 0;
    const double result = std::stod(value, &position);
    if (position != value.size()) {
      throw ParseError(name + " is not numeric");
    }
    if (!std::isfinite(result)) {
      throw ParseError(name + " must be finite");
    }
    return result;
  } catch (const std::invalid_argument &) {
    throw ParseError(name + " is not numeric");
  } catch (const std::out_of_range &) {
    throw ParseError(name + " must be finite");
  }
}

std::uint64_t parse_uint(const std::string & value, const std::string & name)
{
  if (value.empty() || value.front() == '-') {
    throw ParseError(name + " is not an integer");
  }
  try {
    std::size_t position = 0;
    const std::uint64_t result = std::stoull(value, &position, 10);
    if (position != value.size()) {
      throw ParseError(name + " is not an integer");
    }
    return result;
  } catch (const std::invalid_argument &) {
    throw ParseError(name + " is not an integer");
  } catch (const std::out_of_range &) {
    throw ParseError(name + " is out of range");
  }
}

std::string extract_body(const std::string & sentence, bool validate_checksum)
{
  const std::string payload = sentence.substr(1);
  const auto delimiter = payload.rfind('*');
  if (delimiter == std::string::npos) {
    if (validate_checksum) {
      throw ParseError("sentence has no checksum");
    }
    return payload;
  }

  const std::string body = payload.substr(0, delimiter);
  const std::string checksum = payload.substr(delimiter + 1U);
  if (checksum.size() != 2U) {
    throw ParseError("checksum must have two hexadecimal digits");
  }

  unsigned long expected = 0;
  try {
    std::size_t position = 0;
    expected = std::stoul(checksum, &position, 16);
    if (position != checksum.size()) {
      throw ParseError("checksum is not hexadecimal");
    }
  } catch (const std::invalid_argument &) {
    throw ParseError("checksum is not hexadecimal");
  } catch (const std::out_of_range &) {
    throw ParseError("checksum is not hexadecimal");
  }

  std::uint8_t actual = 0;
  for (const unsigned char character : body) {
    actual ^= character;
  }
  if (validate_checksum && actual != expected) {
    throw ParseError("checksum mismatch");
  }
  return body;
}

Dvext parse_dvext(std::vector<std::string> fields)
{
  if (fields.size() == 36U && fields.back().empty()) {
    fields.pop_back();
  }
  if (fields.size() != 35U) {
    throw ParseError("DVEXT needs 35 fields, got " + std::to_string(fields.size()));
  }

  Dvext result{};
  result.bottom_lock = parse_bool(fields[1]);
  const std::uint64_t data_skips = parse_uint(fields[7], "data_skips");
  if (data_skips > std::numeric_limits<std::uint8_t>::max()) {
    throw ParseError("data_skips must fit uint8");
  }
  result.data_skips = static_cast<std::uint8_t>(data_skips);
  result.velocity_up = parse_float(fields[8], "velocity_up");
  result.altitude = parse_float(fields[9], "altitude");
  result.velocity_north = parse_float(fields[10], "velocity_north");
  result.velocity_east = parse_float(fields[11], "velocity_east");
  for (std::size_t index = 0; index < 4U; ++index) {
    result.beam_lock[index] = parse_bool(fields[23U + index]);
    result.beam_velocity[index] = parse_float(fields[27U + index], "beam_velocity");
    result.beam_range[index] = parse_float(fields[31U + index], "beam_range");
  }
  return result;
}

Dvpdl parse_dvpdl(const std::vector<std::string> & fields)
{
  if (fields.size() != 10U) {
    throw ParseError("DVPDL needs 10 fields, got " + std::to_string(fields.size()));
  }

  const std::uint64_t device_time = parse_uint(fields[1], "device_time_us");
  const std::uint64_t delta_time = parse_uint(fields[2], "delta_time_us");
  if (delta_time > std::numeric_limits<std::uint32_t>::max()) {
    throw ParseError("delta_time_us must fit uint32");
  }

  const std::uint64_t confidence = parse_uint(fields[9], "confidence");
  if (confidence > 100U) {
    throw ParseError("confidence must be in the range 0..100");
  }

  Dvpdl result{};
  result.device_time_us = device_time;
  result.delta_time_us = static_cast<std::uint32_t>(delta_time);
  result.confidence = static_cast<std::uint8_t>(confidence);
  for (std::size_t index = 0; index < 3U; ++index) {
    result.angle_delta[index] = parse_float(fields[3U + index], "angle_delta");
    result.position_delta[index] = parse_float(fields[6U + index], "position_delta");
  }
  return result;
}

}  // namespace

Measurement parse_sentence(const std::string & packet, bool validate_checksum)
{
  const std::string sentence = trim(packet);
  if (sentence.empty() || sentence.front() != '$') {
    return std::monostate{};
  }

  const auto fields = split(extract_body(sentence, validate_checksum));
  if (fields.empty() || fields.front().empty()) {
    throw ParseError("sentence has no message identifier");
  }
  if (fields.front() == "DVEXT") {
    return parse_dvext(fields);
  }
  if (fields.front() == "DVPDL") {
    return parse_dvpdl(fields);
  }
  return std::monostate{};
}

}  // namespace dvl75_driver
