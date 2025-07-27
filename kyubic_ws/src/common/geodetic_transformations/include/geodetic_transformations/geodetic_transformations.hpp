#ifndef GEODETIC_CONVERTER_HPP
#define GEODETIC_CONVERTER_HPP

#include <cmath>      // For std::sin, std::cos, std::tan, std::log, std::pow, std::sqrt, M_PI
#include <stdexcept>  // For std::out_of_range

// 緯度経度を表現する構造体
struct LatLon
{
  double latitude;   // 緯度 (度)
  double longitude;  // 経度 (度)
};

// 平面直角座標を表現する構造体
struct PlaneXY
{
  double x;  // X座標 (メートル)
  double y;  // Y座標 (メートル)
};

class GeodeticConverter
{
public:
  // コンストラクタ
  // systemId: 平面直角座標系の系番号 (1-19)
  explicit GeodeticConverter(int systemId);

  // 緯度経度から平面直角座標へ変換
  PlaneXY convertLatLonToPlaneXY(const LatLon & latLon) const;

  // 平面直角座標から緯度経度へ変換
  LatLon convertPlaneXYToLatLon(const PlaneXY & planeXY) const;

private:
  int systemId_;  // 平面直角座標系の系番号

  // 基準点の緯度と経度 (ラジアン)
  double phi0_rad_;     // 基準緯度
  double lambda0_rad_;  // 基準経度

  // GRS80楕円体定数
  static constexpr double A = 6378137.0;                   // 長半径 (メートル)
  static constexpr double F = 298.257222101;               // 扁平率の逆数
  static constexpr double E2 = (2.0 * F - 1.0) / (F * F);  // 離心率の2乗 (e^2)
  static constexpr double E_PRIME2 = E2 / (1.0 - E2);      // 第2離心率の2乗 (e'^2)

  // 内部計算用関数
  double degToRad(double deg) const;
  double radToDeg(double rad) const;
  double calculateM(double phi_rad) const;           // 子午線弧長 M(φ) の計算
  double calculateT(double phi_rad) const;           // 補助関数 T(φ)
  double calculateD(double delta_lambda_rad) const;  // 補助関数 D(λ-λ0)

  // 系番号に基づく基準点設定
  void setReferencePoint(int systemId);
};

#endif  // GEODETIC_CONVERTER_HPP
