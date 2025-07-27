#ifndef GEODETIC_TRANSFORMATIONS_HPP
#define GEODETIC_TRANSFORMATIONS_HPP

namespace GSI {

    /**
     * @struct LatLon
     * @brief 緯度経度を格納する構造体（単位：度）
     */
    struct LatLon {
        double latitude;  // 緯度 (B)
        double longitude; // 経度 (L)
    };

    /**
     * @struct XY
     * @brief 平面直角座標を格納する構造体（単位：メートル）
     */
    struct XY {
        double x; // X座標（真北方向が正）
        double y; // Y座標（真東方向が正）
    };

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

} // namespace GSI

#endif // GEODETIC_TRANSFORMATIONS_HPP
