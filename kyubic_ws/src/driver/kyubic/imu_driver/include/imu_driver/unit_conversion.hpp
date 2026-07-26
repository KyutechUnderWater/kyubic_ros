#ifndef IMU_DRIVER__UNIT_CONVERSION_HPP_
#define IMU_DRIVER__UNIT_CONVERSION_HPP_

#include <cstdint>

namespace driver::imu_driver::g366
{

inline constexpr double GYRO_DEGREE_PER_SECOND_PER_LSB = 1.0 / 66.0;
inline constexpr double ACCEL_MILLI_G_PER_LSB = 1.0 / 4.0;
inline constexpr double MILLI_G_TO_METERS_PER_SECOND_SQUARED = 9.80665 / 1000.0;

/**
 * @brief Convert a raw G366 gyroscope sample to degrees per second.
 */
constexpr double convert_gyro(std::int16_t raw_value)
{
  return raw_value * GYRO_DEGREE_PER_SECOND_PER_LSB;
}

/**
 * @brief Convert a raw G366 accelerometer sample to meters per second squared.
 */
constexpr double convert_accel(std::int16_t raw_value)
{
  return raw_value * ACCEL_MILLI_G_PER_LSB * MILLI_G_TO_METERS_PER_SECOND_SQUARED;
}

}  // namespace driver::imu_driver::g366

#endif  // IMU_DRIVER__UNIT_CONVERSION_HPP_
