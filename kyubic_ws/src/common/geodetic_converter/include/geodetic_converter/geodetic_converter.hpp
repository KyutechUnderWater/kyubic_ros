/**
 * @file geodetic_converter.hpp
 * @brief geodetic to plane converter library
 * @author R.Ohnishi
 * @date 2025/10/02
 *
 * @details 測地座標系 <-> 平面直角座標系 の相互変換
 **************************************************/

#ifndef GEODETIC_CONVERTER_HPP
#define GEODETIC_CONVERTER_HPP

#include <array>
#include <cmath>

/**
 * @namespace common
 * @brief For common
 */
namespace common
{

/** @brief Origin data for the Cartesian coordinate system
 * @details https://www.gsi.go.jp/LAW/heimencho.html
 */
const std::array<std::array<double, 3>, 2> ORIGIN_POINTS_DMS[19] = {
  {{{33, 0, 0}, {129, 30, 0}}},  // I系: 33°00′00″N, 129°30′00″E
  {{{33, 0, 0}, {131, 0, 0}}},   // II系: 33°00′00″N, 131°00′00″E
  {{{36, 0, 0}, {132, 10, 0}}},  // III系: 36°00′00″N, 132°10′00″E
  {{{33, 0, 0}, {133, 30, 0}}},  // IV系: 33°00′00″N, 133°30′00″E
  {{{36, 0, 0}, {134, 20, 0}}},  // V系: 36°00′00″N, 134°20′00″E
  {{{36, 0, 0}, {136, 0, 0}}},   // VI系: 36°00′00″N, 136°00′00″E
  {{{36, 0, 0}, {137, 10, 0}}},  // VII系: 36°00′00″N, 137°10′00″E
  {{{36, 0, 0}, {138, 30, 0}}},  // VIII系: 36°00′00″N, 138°30′00″E
  {{{36, 0, 0}, {139, 50, 0}}},  // IX系: 36°00′00″N, 139°50′00″E
  {{{40, 0, 0}, {140, 50, 0}}},  // X系: 40°00′00″N, 140°50′00″E
  {{{44, 0, 0}, {140, 15, 0}}},  // XI系: 40°00′00″N, 140°15′00″E
  {{{44, 0, 0}, {142, 15, 0}}},  // XII系: 44°00′00″N, 142°15′00″E
  {{{44, 0, 0}, {144, 15, 0}}},  // XIII系: 44°00′00″N, 144°15′00″E
  {{{26, 0, 0}, {142, 0, 0}}},   // XIV系: 26°00′00″N, 142°00′00″E
  {{{26, 0, 0}, {127, 30, 0}}},  // XV系: 26°00′00″N, 127°30′00″E
  {{{26, 0, 0}, {124, 0, 0}}},   // XVI系: 26°00′00″N, 124°00′00″E
  {{{26, 0, 0}, {131, 0, 0}}},   // XVII系: 26°00′00″N, 131°00′00″E
  {{{20, 0, 0}, {136, 0, 0}}},   // XVIII系: 20°00′00″N, 136°00′00″E
  {{{26, 0, 0}, {154, 0, 0}}}    // XIX系: 26°00′00″N, 154°00′00″E
};

/**
 * @brief Structure of latitude longitude
 */
struct Geodetic
{
  double latitude;   // 緯度 (deg)
  double longitude;  // 経度 (deg)
  double meridian_convergence;
  double scale_coefficient;
};

/**
 * @brief Structure of Cartesian coordinate system
 */
struct PlaneXY
{
  double x;  // X座標 (meter)
  double y;  // Y座標 (meter)
  double meridian_convergence;
  double scale_coefficient;
};

/**
 * @brief custom power function
 * @param base base
 * @param exp exponent
 * @details constexpr pow function
 */
static constexpr double power(double base, int exp)
{
  return (exp == 0) ? 1.0 : base * power(base, exp - 1);
}

/**
 * @brief GeodeticConverter class
 */
class GeodeticConverter
{
public:
  /**
   * @brief Set System ID
   * @param systemId Cartesian coordinate system (1-19)
   */
  explicit GeodeticConverter(int systemId);

  /**
   * @brief Get longitude and latitude that serve as the origin of the Cartesian coordinate system
   * @return origin(latitude and longitude)
   */
  Geodetic getOrigin();

  /**
   * @brief Conversion from latitude and longitude to Cartesian coordinate system
   * @param latLon latitude and longitude
   * @return x, y
   * @details https://vldb.gsi.go.jp/sokuchi/surveycalc/surveycalc/bl2xyf.html
   */
  PlaneXY geo2xy(const Geodetic & latLon) const;

  /**
   * @brief Conversion from Cartesian coordinate system to latitude and longitude
   * @param planeXY xy coordinate
   * @return latitude and longitude
   * @details https://vldb.gsi.go.jp/sokuchi/surveycalc/surveycalc/xy2blf.html
   */
  Geodetic xy2geo(const PlaneXY & planeXY) const;

  /**
   * @brief degrees-minutes-seconds to decimal degrees converter
   * @param dms 3d array(degrees, minutes, seconds)
   * @return decimal degrees
   */
  double dms2deg(const std::array<double, 3> & dms);

  /**
   * @brief decimal degrees to degrees-minutes-seconds converter
   * @param deg decimal degrees
   * @return 3d array(degrees, minutes, seconds)
   */
  std::array<double, 3> deg2dmg(const double & deg) const;

  /**
   * @brief degree to radian converter
   * @param deg degree
   * @return radian
   */
  double deg2rad(double deg) const;

  /**
   * @brief radian to degree converter
   * @param rad radian
   * @return degrees
   */
  double rad2deg(double rad) const;

private:
  int systemId_;

  // 基準点の緯度と経度 (ラジアン)
  double phi0_rad_;     // 基準緯度
  double lambda0_rad_;  // 基準経度
  double S_bar_phi0_;

  // GRS80楕円体定数
  static constexpr double a = 6378137.0;      // 長半径 (メートル)
  static constexpr double F = 298.257222101;  // 扁平率の逆数
  static constexpr double m0 = 0.9999;        // 平面直角座標系のX軸上における縮尺係数
  static constexpr double n = 1 / (2 * F - 1);
  static constexpr double n2 = power(n, 2);
  static constexpr double n3 = power(n, 3);
  static constexpr double n4 = power(n, 4);
  static constexpr double n5 = power(n, 5);
  static constexpr double alpha[6] = {
    0.0,
    n / 2 - 2 * n2 / 3 + 5 * n3 / 16 + 41 * n4 / 180 - 127 * n5 / 288,
    13 * n2 / 48 - 3 * n3 / 5 + 557 * n4 / 1440 + 281 * n5 / 630,
    61 * n3 / 240 - 103 * n4 / 140 + 15061 * n5 / 26880,
    49561 * n4 / 161280 - 179 * n5 / 168,
    34729 * n5 / 80640};
  static constexpr double beta[6] = {
    0.0,
    n / 2 - 2 * n2 / 3 + 37 * n3 / 96 - n4 / 360 - 81 * n5 / 512,
    n2 / 48 + n3 / 15 - 437 * n4 / 1440 + 46 * n5 / 105,
    17 * n3 / 480 - 37 * n4 / 840 - 209 * n5 / 4480,
    4397 * n4 / 161280 - 11 * n5 / 504,
    4583 * n5 / 161280};
  static constexpr double delta[6] = {
    0.0,
    -2 * n2 / 3 - 2 * n3 / 3 + 4 * n4 / 9 + 2 * n5 / 9,
    n2 / 3 - 4 * n3 / 15 - 23 * n4 / 45 + 68 * n5 / 45,
    2 * n3 / 5 - 24 * n4 / 35 - 46 * n5 / 35,
    83 * n4 / 126 - 80 * n5 / 63,
    52 * n5 / 45};
  static constexpr double A[6] = {1 + n2 / 4 + n4 / 64,    -3 * (n - n3 / 8 - n5 / 64) / 2,
                                  15 * (n2 - n4 / 4) / 16, -35 * (n3 - 5 * n5 / 16) / 48,
                                  315 * n4 / 512,          -693 * n5 / 1280};
  static constexpr double A_bar = m0 * a * A[0] / (1 + n);

  // 内部計算用関数
  double calc_S_bar_phi0(const double & phi0_rad) const;

  /**
   * @brief Set reference point(Cartesian coordinate system)
   */
  void setReferencePoint(int systemId);
};

}  // namespace common

#endif  // GEODETIC_CONVERTER_HPP
