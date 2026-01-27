/**
 * @file pd0_parser.hpp
 * @brief Parser for Teledyne RDI PD0 Data Format
 * @author R.Ohnishi
 * @date 2025/12/15
 *
 * @details DVL(Path Finder) のPD0データをデコードする
 ****************************************************/

#ifndef _PD0_PARSER_HPP
#define _PD0_PARSER_HPP

#include <cstddef>

#include "dvl_driver/pd0_types.hpp"

namespace driver::dvl_driver::path_finder::pd0
{

/**
 * @class Pd0Parser
 * @brief Class to parse PD0 binary data packets
 */
class Pd0Parser
{
public:
  Pd0Parser();
  ~Pd0Parser();

  /**
   * @brief Parse a raw byte buffer containing a PD0 ensemble
   * @param buffer Pointer to the raw binary data
   * @param length Length of the data in bytes
   * @param out_data Reference to store the parsed PD0 ensemble data
   * @return true if the packet is valid and successfully parsed
   */
  bool parse(const unsigned char * buffer, size_t length, Pd0Ensemble & out_data);
};

}  // namespace driver::dvl_driver::path_finder::pd0

#endif  // _PD0_PARSER_HPP
