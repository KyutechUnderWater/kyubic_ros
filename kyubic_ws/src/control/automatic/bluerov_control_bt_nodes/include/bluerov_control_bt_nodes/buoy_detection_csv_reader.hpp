#ifndef _BUOY_DETECTION_CSV_READER_HPP
#define _BUOY_DETECTION_CSV_READER_HPP

#include <string>

namespace bluerov_control_bt_nodes
{
namespace buoy_detection_csv_reader
{

/**
 * perception/buoy_detector (buoy_detector_node.py) が書き出す固定CSVの絶対パス。
 * CSV_PATH = Path.home()/"kyubic_ws"/.../"path_planner"/"assets"/"iwakuni"/"bouy_point.csv"
 * (buoy_detector_node.py参照)。コンテナ内ユーザーは"ros"なのでホームは/home/ros。
 * 列: frame_index,timestamp_sec,detected,yaw_valid,position_valid,x,y,z,yaw,
 *     horizontal_offset,vertical_offset,confidence
 */
extern const char * const kBuoyDetectionCsvPath;

// buoy_interfaces/msg/BuoyRelativePositionと同じ内容をCSVの最終行から読んだもの。
struct BuoyDetectionRow
{
  bool detected = false;
  bool yaw_valid = false;
  bool position_valid = false;
  double x_m = 0.0;
  double y_m = 0.0;
  double z_m = 0.0;
  double yaw_rad = 0.0;
  double normalized_horizontal_offset = 0.0;
  double normalized_vertical_offset = 0.0;
  double confidence = 0.0;
};

// pathの最終更新時刻がmax_age_secより古い場合(buoy_detector未起動・停止中)、または最終行を
// 読めない/パースできない場合はfalseを返す。成功時はoutへCSV最終行の値を入れてtrueを返す。
bool ReadLatest(const std::string & path, double max_age_sec, BuoyDetectionRow & out);

}  // namespace buoy_detection_csv_reader
}  // namespace bluerov_control_bt_nodes

#endif
