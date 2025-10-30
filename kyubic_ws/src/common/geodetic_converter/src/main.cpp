/**
 * @file main.cpp
 * @brief geodetic to plane converter example
 * @author R.Ohnishi
 * @date 2025/10/02
 *
 * @details 測地座標系 <-> 平面直角座標系 の相互変換のサンプル
 ************************************************************/

#include "geodetic_converter/geodetic_converter.hpp"

#include <array>
#include <iomanip>
#include <iostream>

int main()
{
  // --- テスト用の緯度経度 ---
  // 例: 新宿駅付近
  common::Geodetic latlon = {35.69000000, 139.69200000, 0.0, 0.0};

  // --- 座標系の原点 ---
  // 平面直角座標系 第IX系（関東地方）の原点
  common::GeodeticConverter geo_converter(9);

  // --- 緯度経度から座標への変換 ---
  std::cout << "--- latitude and longitude -> Cartisian coordinate ---" << std::endl;
  std::cout << std::fixed << std::setprecision(8);
  std::cout << "Input latitude: " << latlon.latitude << " [deg]" << std::endl;
  std::cout << "Input longitude: " << latlon.longitude << " [deg]" << std::endl;

  common::PlaneXY planexy = geo_converter.geo2xy(latlon);

  std::cout << "\nresults:" << std::endl;
  std::cout << "X-coordinate: " << planexy.x << " [m]" << std::endl;
  std::cout << "Y-coordinate: " << planexy.y << " [m]" << std::endl;
  std::array<double, 3> lat = geo_converter.deg2dmg(planexy.meridian_convergence);
  std::cout << "子午線収差角: " << lat[0] << "°" << lat[1] << "'" << lat[2] << "\"" << ", "
            << planexy.meridian_convergence << " [deg]" << std::endl;
  std::cout << "縮尺係数: " << planexy.scale_coefficient << " [-]" << std::endl;

  std::cout << "\n---------------------------------------------\n" << std::endl;

  common::PlaneXY plane = {planexy.x, planexy.y, 0.0, 0.0};

  // --- 座標から緯度経度への逆変換 ---
  std::cout << "--- 平面直角座標 -> 緯度経度 ---" << std::endl;
  std::cout << "Input X-coordinate: " << plane.x << " [m]" << std::endl;
  std::cout << "Input Y-coordinate: " << plane.y << " [m]" << std::endl;

  common::Geodetic geo = geo_converter.xy2geo(plane);

  std::cout << "\nresult of inverse convert:" << std::endl;
  lat = geo_converter.deg2dmg(geo.latitude);
  std::cout << "latitude: " << lat[0] << "°" << lat[1] << "'" << lat[2] << "\"" << ", "
            << geo.latitude << " [deg]" << std::endl;
  lat = geo_converter.deg2dmg(geo.longitude);
  std::cout << "longitude: " << lat[0] << "°" << lat[1] << "'" << lat[2] << "\"" << ", "
            << geo.longitude << " [deg]" << std::endl;
  lat = geo_converter.deg2dmg(geo.meridian_convergence);
  std::cout << "子午線収差角: " << lat[0] << "°" << lat[1] << "'" << lat[2] << "\"" << ", "
            << geo.meridian_convergence << " [deg]" << std::endl;
  std::cout << "縮尺係数: " << geo.scale_coefficient << " [-]" << std::endl;

  return 0;
}
