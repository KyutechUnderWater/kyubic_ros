#include "bluerov_control_bt_nodes/buoy_detection_csv_reader.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

namespace bluerov_control_bt_nodes
{
namespace buoy_detection_csv_reader
{

const char * const kBuoyDetectionCsvPath =
  "/home/ros/kyubic_ws/src/control/automatic/planning/path_planner/assets/iwakuni/bouy_point.csv";

namespace
{
std::vector<std::string> SplitCsvLine(const std::string & line)
{
  std::vector<std::string> tokens;
  std::stringstream ss(line);
  std::string token;
  while (std::getline(ss, token, ',')) {
    tokens.push_back(token);
  }
  return tokens;
}

// buoy_detector_node.pyが画像フレームごとに追記し続ける大きなCSVを毎tick全読込しないよう、
// ファイル末尾の数KBだけを読んで最後のデータ行を取り出す。
bool ReadLastCsvLine(const std::string & path, std::string & out_line)
{
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    return false;
  }
  file.seekg(0, std::ios::end);
  const std::streamoff file_size = file.tellg();
  if (file_size <= 0) {
    return false;
  }
  const std::streamoff read_size = std::min<std::streamoff>(file_size, 8192);
  file.seekg(file_size - read_size);
  std::string tail(static_cast<size_t>(read_size), '\0');
  file.read(&tail[0], read_size);

  while (!tail.empty() && (tail.back() == '\n' || tail.back() == '\r')) {
    tail.pop_back();
  }
  const size_t last_newline = tail.find_last_of('\n');
  out_line = (last_newline == std::string::npos) ? tail : tail.substr(last_newline + 1);
  return !out_line.empty();
}

bool ParseRow(const std::string & line, BuoyDetectionRow & row)
{
  const std::vector<std::string> tokens = SplitCsvLine(line);
  // frame_index,timestamp_sec,detected,yaw_valid,position_valid,x,y,z,yaw,
  // horizontal_offset,vertical_offset,confidence
  if (tokens.size() != 12 || tokens[0] == "frame_index") {
    // 列数不一致、またはヘッダー行のみでまだデータ行がない。
    return false;
  }
  row.detected = (tokens[2] == "True");
  row.yaw_valid = (tokens[3] == "True");
  row.position_valid = (tokens[4] == "True");
  try {
    if (row.position_valid) {
      row.x_m = std::stod(tokens[5]);
      row.y_m = std::stod(tokens[6]);
      row.z_m = std::stod(tokens[7]);
    }
    if (row.yaw_valid) {
      row.yaw_rad = std::stod(tokens[8]);
    }
    if (!tokens[9].empty()) {
      row.normalized_horizontal_offset = std::stod(tokens[9]);
    }
    if (!tokens[10].empty()) {
      row.normalized_vertical_offset = std::stod(tokens[10]);
    }
    row.confidence = tokens[11].empty() ? 0.0 : std::stod(tokens[11]);
  } catch (const std::exception &) {
    return false;
  }
  return true;
}
}  // namespace

bool ReadLatest(const std::string & path, double max_age_sec, BuoyDetectionRow & out)
{
  std::error_code ec;
  const auto last_write_time = std::filesystem::last_write_time(path, ec);
  if (ec) {
    // buoy_detector_node.pyがまだCSVを作成していない。
    return false;
  }
  const auto age_sec =
    std::chrono::duration<double>(std::chrono::file_clock::now() - last_write_time).count();
  if (age_sec > max_age_sec) {
    // CSVが更新されていない(ノード停止・フレーム落ちなど)。
    return false;
  }

  std::string last_line;
  if (!ReadLastCsvLine(path, last_line)) {
    return false;
  }
  return ParseRow(last_line, out);
}

}  // namespace buoy_detection_csv_reader
}  // namespace bluerov_control_bt_nodes
