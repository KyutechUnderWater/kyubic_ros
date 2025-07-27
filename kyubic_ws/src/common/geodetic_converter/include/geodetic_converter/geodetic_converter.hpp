#ifndef GEODETIC_CONVERTER_HPP
#define GEODETIC_CONVERTER_HPP

#include <array>

namespace GSI
{

// 平面直角座標系の原点データ
// ref: https://www.gsi.go.jp/LAW/heimencho.html
const std::array<std::array<int, 3>, 2> ORIGIN_POINTS_DMS[19] = {
  {{{33, 0, 0}, {129, 30, 0}}},   // I系: 33°00′00″N, 129°30′00″E
  {{{33, 0, 0}, {131, 0, 0}}},    // II系: 33°00′00″N, 131°00′00″E
  {{{36, 0, 0}, {132, 10, 30}}},  // III系: 36°00′00″N, 132°10′30″E
  {{{33, 0, 0}, {133, 0, 0}}},    // IV系: 33°00′00″N, 133°00′00″E
  {{{36, 0, 0}, {134, 0, 0}}},    // V系: 36°00′00″N, 134°00′00″E
  {{{36, 0, 0}, {136, 0, 0}}},    // VI系: 36°00′00″N, 136°00′00″E
  {{{36, 0, 0}, {137, 0, 0}}},    // VII系: 36°00′00″N, 137°00′00″E
  {{{36, 0, 0}, {138, 0, 0}}},    // VIII系: 36°00′00″N, 138°00′00″E
  {{{36, 0, 0}, {139, 0, 0}}},    // IX系: 36°00′00″N, 139°00′00″E
  {{{40, 0, 0}, {140, 0, 0}}},    // X系: 40°00′00″N, 140°00′00″E
  {{{40, 0, 0}, {142, 0, 0}}},    // XI系: 40°00′00″N, 142°00′00″E
  {{{44, 0, 0}, {142, 0, 0}}},    // XII系: 44°00′00″N, 142°00′00″E
  {{{44, 0, 0}, {144, 0, 0}}},    // XIII系: 44°00′00″N, 144°00′00″E
  {{{26, 0, 0}, {127, 30, 0}}},   // XIV系: 26°00′00″N, 127°30′00″E
  {{{26, 0, 0}, {124, 0, 0}}},    // XV系: 26°00′00″N, 124°00′00″E
  {{{26, 0, 0}, {141, 0, 0}}},    // XVI系: 26°00′00″N, 141°00′00″E
  {{{26, 0, 0}, {157, 0, 0}}},    // XVII系: 26°00′00″N, 157°00′00″E
  {{{20, 0, 0}, {136, 0, 0}}},    // XVIII系: 20°00′00″N, 136°00′00″E
  {{{26, 0, 0}, {130, 0, 0}}}     // XIX系: 26°00′00″N, 130°00′00″E
};

/**
     * @struct LatLon
     * @brief 緯度経度を格納する構造体（単位：度）
     */
struct LatLon
{
  double latitude;   // 緯度 (B)
  double longitude;  // 経度 (L)
};

/**
     * @struct XY
     * @brief 平面直角座標を格納する構造体（単位：メートル）
     */
struct XY
{
  double x;  // X座標（真北方向が正）
  double y;  // Y座標（真東方向が正）
};

double dms2deg(const std::array<int, 3> & dms);

LatLon getOriginPoint(int systemId);

/**
     * @brief [GSI方式] 緯度経度から平面直角座標へ変換します。
     * @param lat 緯度（度）
     * @param lon 経度（度）
     * @param origin_lat 座標系の原点の緯度（度）
     * @param origin_lon 座標系の原点の経度（度）
     * @return XY 平面直角座標 (x, y)
     */
XY bl2xy(double lat, double lon, double origin_lat, double origin_lon);

/**
     * @brief [GSI方式] 平面直角座標から緯度経度へ変換します。
     * @param x X座標（メートル）
     * @param y Y座標（メートル）
     * @param origin_lat 座標系の原点の緯度（度）
     * @param origin_lon 座標系の原点の経度（度）
     * @return LatLon 緯度経度 (latitude, longitude)
     */
LatLon xy2bl(double x, double y, double origin_lat, double origin_lon);

}  // namespace GSI

#endif  // GEODETIC_TRANSFORMATIONS_HPP
