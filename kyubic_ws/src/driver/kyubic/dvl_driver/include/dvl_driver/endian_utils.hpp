/**
 * @file endian_utils.hpp
 * @brief Common Endian Conversion Utilities
 * @author R.Ohnishi
 * @date 2025/12/15
 *
 * @details bitの結合をする
 * **************************************/

#ifndef _ENDIAN_UTILS_HPP
#define _ENDIAN_UTILS_HPP

#include <cstdint>

namespace driver::dvl_driver::utils
{

/**
 * @brief Extract 8-bit unsigned integer
 */
inline uint8_t get_u8(const unsigned char * ptr) { return ptr[0]; }

/**
 * @brief Extract 16-bit unsigned integer from buffer (Little Endian)
 */
inline uint16_t get_u16(const unsigned char * ptr)
{
  return static_cast<uint16_t>(ptr[0]) | (static_cast<uint16_t>(ptr[1]) << 8);
}

/**
 * @brief Extract 16-bit signed integer from buffer (Little Endian)
 */
inline int16_t get_s16(const unsigned char * ptr) { return static_cast<int16_t>(get_u16(ptr)); }

/**
 * @brief Extract 32-bit unsigned integer from buffer (Little Endian)
 */
inline uint32_t get_u32(const unsigned char * ptr)
{
  return static_cast<uint32_t>(ptr[0]) | (static_cast<uint32_t>(ptr[1]) << 8) |
         (static_cast<uint32_t>(ptr[2]) << 16) | (static_cast<uint32_t>(ptr[3]) << 24);
}

/**
 * @brief Extract 32-bit signed integer from buffer (Little Endian)
 */
inline int32_t get_s32(const unsigned char * ptr) { return static_cast<int32_t>(get_u32(ptr)); }

}  // namespace driver::dvl_driver::utils

#endif  // _ENDIAN_UTILS_HPP
