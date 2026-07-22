#include "dvl75_driver/dvl75_parser.hpp"
#include "gtest/gtest.h"

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <variant>

namespace
{

std::string sentence(const std::string & body)
{
  std::uint8_t checksum = 0;
  for (const unsigned char character : body) {
    checksum ^= character;
  }
  std::stringstream stream;
  stream << '$' << body << '*' << std::uppercase << std::hex << std::setfill('0') << std::setw(2)
         << static_cast<unsigned int>(checksum);
  return stream.str();
}

TEST(Dvl75Parser, ParsesDvpdl)
{
  const auto result =
    dvl75_driver::parse_sentence(sentence("DVPDL,101234000,50000,0.1,-0.2,0.3,1,2,3,95"), true);

  ASSERT_TRUE(std::holds_alternative<dvl75_driver::Dvpdl>(result));
  const auto & measurement = std::get<dvl75_driver::Dvpdl>(result);
  EXPECT_EQ(measurement.device_time_us, 101234000U);
  EXPECT_EQ(measurement.delta_time_us, 50000U);
  EXPECT_DOUBLE_EQ(measurement.position_delta[0], 1.0);
  EXPECT_DOUBLE_EQ(measurement.position_delta[2], 3.0);
  EXPECT_EQ(measurement.confidence, 95U);
}

TEST(Dvl75Parser, ParsesDvextWithTrailingComma)
{
  const auto result = dvl75_driver::parse_sentence(
    sentence("DVEXT,T,V,0000,1.0,2.0,3.0,4,0.1,5.0,0.2,0.3,35.0,139.0,0.05,"
             "1.0,0.0,0.0,0.0,6,7,8,9,T,F,T,T,0.4,0.5,0.6,0.7,8,9,10,11,"),
    true);

  ASSERT_TRUE(std::holds_alternative<dvl75_driver::Dvext>(result));
  const auto & measurement = std::get<dvl75_driver::Dvext>(result);
  EXPECT_TRUE(measurement.bottom_lock);
  EXPECT_EQ(measurement.data_skips, 4U);
  EXPECT_TRUE(measurement.beam_lock[0]);
  EXPECT_FALSE(measurement.beam_lock[1]);
  EXPECT_DOUBLE_EQ(measurement.beam_range[3], 11.0);
}

TEST(Dvl75Parser, RejectsInvalidChecksum)
{
  EXPECT_THROW(
    dvl75_driver::parse_sentence("$DVPDL,1,1,0,0,0,0,0,0,100*00", true), dvl75_driver::ParseError);
}

TEST(Dvl75Parser, RejectsNegativeUnsignedValue)
{
  EXPECT_THROW(
    dvl75_driver::parse_sentence(sentence("DVPDL,1,-1,0,0,0,0,0,0,100"), true),
    dvl75_driver::ParseError);
}

TEST(Dvl75Parser, RejectsTrailingFloatCharacters)
{
  EXPECT_THROW(
    dvl75_driver::parse_sentence(sentence("DVPDL,1,1,0,0,0,1x,0,0,100"), true),
    dvl75_driver::ParseError);
}

TEST(Dvl75Parser, IgnoresNonNmeaInformation)
{
  const auto result = dvl75_driver::parse_sentence("DVL system boot complete", true);
  EXPECT_TRUE(std::holds_alternative<std::monostate>(result));
}

}  // namespace
