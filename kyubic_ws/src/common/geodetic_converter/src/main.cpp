#include "geodetic_converter/geodetic_converter.hpp"

#include <iomanip>
#include <iostream>

int main()
{
  // --- テスト用の緯度経度 ---
  // 例: 新宿駅付近
  double test_lat = 35.69000000;
  double test_lon = 139.69200000;

  // --- 座標系の原点 ---
  // 平面直角座標系 第IX系（関東地方）の原点
  double origin_lat = 36.0;
  double origin_lon = 139.0 + 50.0 / 60.0;  // 139度50分
  // 例えば第Ⅲ系(山口県　島根県　広島県)の場合は
  //double origin_rat = 33.0;
  //double origin_lon = 132.0 + 10.0 / 60.0; //(132度10分)と宣言する
  //第Ⅱ系(長崎以外の九州)
  //double origin_rat = 33.0;
  //double origin_lon = 132.0 + 10.0 / 60.0; //(132度10分)と宣言する
  // --- 緯度経度から座標への変換 ---
  std::cout << "--- 緯度経度 -> 平面直角座標 ---" << std::endl;
  std::cout << std::fixed << std::setprecision(8);
  std::cout << "入力緯度 (B): " << test_lat << " [deg]" << std::endl;
  std::cout << "入力経度 (L): " << test_lon << " [deg]" << std::endl;

  GSI::XY result_xy = GSI::bl2xy(test_lat, test_lon, origin_lat, origin_lon);

  std::cout << "\n変換結果:" << std::endl;
  std::cout << "X座標: " << result_xy.x << " [m]" << std::endl;
  std::cout << "Y座標: " << result_xy.y << " [m]" << std::endl;

  std::cout << "\n---------------------------------------------\n" << std::endl;

  // --- 座標から緯度経度への逆変換 ---
  std::cout << "--- 平面直角座標 -> 緯度経度 ---" << std::endl;
  std::cout << "入力X座標: " << result_xy.x << " [m]" << std::endl;
  std::cout << "入力Y座標: " << result_xy.y << " [m]" << std::endl;

  GSI::LatLon result_bl = GSI::xy2bl(result_xy.x, result_xy.y, origin_lat, origin_lon);

  std::cout << "\n逆変換の結果:" << std::endl;
  std::cout << "緯度 (B): " << result_bl.latitude << " [deg]" << std::endl;
  std::cout << "経度 (L): " << result_bl.longitude << " [deg]" << std::endl;
  std::cout << "(入力値とほぼ同じ値に戻れば成功です)" << std::endl;

  return 0;
}
