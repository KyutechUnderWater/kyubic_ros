/**
 * @file pd5_parser.hpp
 * @brief Parser for Teledyne RDI PD4/PD5 Data Format
 * @author R.Ohnishi
 * @date 2025/12/15
 *
 * @details DVL(Path Finder) のPD4/PD5データをデコードする
 ********************************************************/

#ifndef _PD5_PARSER_HPP
#define _PD5_PARSER_HPP

#include <cstddef>

#include "dvl_driver/pd5_types.hpp"

namespace dvl_driver::path_finder::pd5
{

/**
 * @class Pd5Parser
 * @brief Class to parse PD4/PD5 binary data packets
 */
class Pd5Parser
{
public:
  Pd5Parser();
  ~Pd5Parser();

  /**
   * @brief Parse a raw byte buffer containing a PD4 or PD5 ensemble
   * @param buffer Pointer to the raw binary data
   * @param length Length of the data in bytes
   * @param out_data Reference to store the parsed PD5 ensemble data
   * @return true if the packet is valid and successfully parsed
   */
  bool parse(const unsigned char * buffer, size_t length, Pd5Ensemble & out_data);
};

}  // namespace dvl_driver::path_finder::pd5

#endif  // _PD5_PARSER_HPP
