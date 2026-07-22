#include <gtest/gtest.h>

#include "imu_driver/unit_conversion.hpp"

namespace driver::imu_driver::g366
{

TEST(UnitConversion, ConvertsGyroscopeToDegreesPerSecond)
{
  EXPECT_DOUBLE_EQ(convert_gyro(66), 1.0);
  EXPECT_DOUBLE_EQ(convert_gyro(-66), -1.0);
}

TEST(UnitConversion, ConvertsAccelerationToMetersPerSecondSquared)
{
  EXPECT_NEAR(convert_accel(4), 0.00980665, 1e-12);
  EXPECT_NEAR(convert_accel(-4), -0.00980665, 1e-12);
}

}  // namespace driver::imu_driver::g366
