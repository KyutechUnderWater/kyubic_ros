#include "bluerov_control_bt_nodes/waypoint_csv_writer.hpp"

#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace bluerov_control_bt_nodes
{
namespace waypoint_csv_writer
{

namespace
{
std::string JoinCsvRow(const std::vector<std::string> & fields)
{
  std::ostringstream oss;
  for (size_t i = 0; i < fields.size(); ++i) {
    if (i > 0) oss << ",";
    oss << fields[i];
  }
  return oss.str();
}

std::string FormatDouble(double value)
{
  std::ostringstream oss;
  oss.precision(4);
  oss << std::fixed << value;
  return oss.str();
}
}  // namespace

std::string ResolvePath(const rclcpp::Logger & logger, const std::string & csv_file_path)
{
  if (csv_file_path.empty() || csv_file_path[0] == '/') {
    return csv_file_path;
  }
  try {
    const std::string package_share_dir =
      ament_index_cpp::get_package_share_directory("path_planner");
    return package_share_dir + "/" + csv_file_path;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "waypoint_csv_writer: path_plannerの解決に失敗: %s", e.what());
    return csv_file_path;
  }
}

void WriteSingleCheckpoint(
  const std::string & resolved_path, double x, double y, double z, uint8_t z_mode, double roll,
  double yaw)
{
  const std::filesystem::path final_path(resolved_path);
  std::filesystem::create_directories(final_path.parent_path());

  const std::vector<std::string> header = {
    "parameter_label",
    "value",
    "x",
    "y",
    "z",
    "z_mode",
    "roll",
    "yaw",
    "wait_ms",
    "fine",
    "blank",
    "catmull_x",
    "catmull_y",
    "catmull_z",
    "catmull_z_mode",
    "catmull_roll",
    "catmull_yaw",
    "catmull_wait_ms",
    "catmull_fine"};
  const std::vector<std::pair<std::string, std::string>> params = {
    {"checkpoint_end_row", "1"},
    {"catmull_rom", "0"},
    {"catmull_end_row", "0"},
    {"catmull_density", "0"},
    {"catmull_min_distance", "0.0"},
    {"catmull_orient_LERP", "0"},
    {"timeout_sec", "30"}};

  std::ostringstream body;
  body << JoinCsvRow(header) << "\n";
  for (const auto & [label, value] : params) {
    body << JoinCsvRow(
              {label, value, "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""})
         << "\n";
  }
  body << JoinCsvRow(
            {"", "", FormatDouble(x), FormatDouble(y), FormatDouble(z), std::to_string(z_mode),
             FormatDouble(roll), FormatDouble(yaw),
             "0",  // wait_ms
             "0",  // fine
             "", "", "", "", "", "", "", "", ""})
       << "\n";

  const std::string tmp_path = resolved_path + ".tmp." + std::to_string(getpid());
  {
    std::ofstream ofs(tmp_path, std::ios::trunc);
    if (!ofs.is_open()) {
      throw std::runtime_error("一時ファイルを開けません: " + tmp_path);
    }
    ofs << body.str();
    if (!ofs.good()) {
      throw std::runtime_error("一時ファイルへの書き込みに失敗しました: " + tmp_path);
    }
  }
  std::filesystem::rename(tmp_path, final_path);
}

}  // namespace waypoint_csv_writer
}  // namespace bluerov_control_bt_nodes
