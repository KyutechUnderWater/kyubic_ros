#include "geodetic_transformations/geodetic_transformations.hpp"

#include <iomanip>  // For std::fixed, std::setprecision
#include <iostream>

int main()
{
  // 例: 日本の平面直角座標系IX系 (東京近辺)
  // 基準点: 北緯36度00分00秒、東経139度00分00秒
  int systemId = 9;
  GeodeticConverter converter(systemId);

  // 緯度経度 -> 平面直角座標 変換例
  // 東京都庁の概略位置
  LatLon tokyoStationLatLon = {35.6895, 139.6917};  // 新宿駅付近

  PlaneXY tokyoStationXY = converter.convertLatLonToPlaneXY(tokyoStationLatLon);

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "--- 緯度経度から平面直角座標へ ---" << std::endl;
  std::cout << "入力緯度: " << tokyoStationLatLon.latitude
            << "度, 経度: " << tokyoStationLatLon.longitude << "度" << std::endl;
  std::cout << "変換結果 X: " << tokyoStationXY.x << " m, Y: " << tokyoStationXY.y << " m"
            << std::endl;
  std::cout << std::endl;

  // 平面直角座標 -> 緯度経度 変換例
  // 上で計算したXY座標を元に戻す
  PlaneXY calculatedXY = {tokyoStationXY.x, tokyoStationXY.y};
  LatLon convertedLatLon = converter.convertPlaneXYToLatLon(calculatedXY);

  std::cout << "--- 平面直角座標から緯度経度へ ---" << std::endl;
  std::cout << "入力X: " << calculatedXY.x << " m, Y: " << calculatedXY.y << " m" << std::endl;
  std::cout << "変換結果 緯度: " << convertedLatLon.latitude
            << "度, 経度: " << convertedLatLon.longitude << "度" << std::endl;
  std::cout << std::endl;

  // 別の系でのテスト例: 福岡県北九州市 (III系が近いが、I系で計算)
  // 北九州市役所の概略位置
  // I系: 基準点 北緯33度00分00秒、東経129度30分00秒
  try {
    GeodeticConverter kitakyushuConverter(1);  // I系

    LatLon kitakyushuLatLon = {33.8824, 130.8753};  // 北九州市役所付近

    PlaneXY kitakyushuXY = kitakyushuConverter.convertLatLonToPlaneXY(kitakyushuLatLon);

    std::cout << "--- 北九州市 (I系) 変換例 ---" << std::endl;
    std::cout << "入力緯度: " << kitakyushuLatLon.latitude
              << "度, 経度: " << kitakyushuLatLon.longitude << "度" << std::endl;
    std::cout << "変換結果 X: " << kitakyushuXY.x << " m, Y: " << kitakyushuXY.y << " m"
              << std::endl;
    std::cout << std::endl;

    LatLon kitakyushuConvertedLatLon = kitakyushuConverter.convertPlaneXYToLatLon(kitakyushuXY);
    std::cout << "逆変換結果 緯度: " << kitakyushuConvertedLatLon.latitude
              << "度, 経度: " << kitakyushuConvertedLatLon.longitude << "度" << std::endl;

  } catch (const std::out_of_range & e) {
    std::cerr << "エラー: " << e.what() << std::endl;
  }

  return 0;
}
