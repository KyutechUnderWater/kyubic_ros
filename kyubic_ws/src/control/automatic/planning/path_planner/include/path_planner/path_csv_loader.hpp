/**
 * @file path_csv_parser.hpp
 * @brief Parse csv file with route infomation
 * @author R.Ohnishi
 * @date 2025/07/19
 *
 * @details 経路情報を持つcsvファイルの構文解析
 **********************************************/

#ifndef _PATH_CSV_PARSER_HPP
#define _PATH_CSV_PARSER_HPP

#include <array>
#include <cstddef>
#include <geodetic_converter/geodetic_converter.hpp>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace planner
{

class ParamData
{
public:
  size_t checkpoint_end_row = 0;
  bool catmull_rom = false;
  size_t catmull_end_row = 0;
  int catmull_density = 0;
  double catmull_min_distance = 0.0;
  bool catmull_orient_LERP = false;
  int timeout_sec = 0;

  bool use_geodetic_coords = false;
  double origin_lat = 0.0;
  double origin_lon = 0.0;
  int system_id = 0;

  void print() const
  {
    std::cout << "  checkpoint_end_row: " << checkpoint_end_row << std::endl;
    std::cout << "  catmull_rom: " << catmull_rom << std::endl;
    std::cout << "  catmull_end_row: " << catmull_end_row << std::endl;
    std::cout << "  catmull_density: " << catmull_density << std::endl;
    std::cout << "  catmull_min_distance: " << catmull_density << std::endl;
    std::cout << "  catmull_orient_LERP: " << catmull_orient_LERP << std::endl;
    std::cout << "  timeout_sec: " << timeout_sec << std::endl;
    std::cout << "  use_geodetic_coords: " << use_geodetic_coords << std::endl;
    std::cout << "  origin_lat: " << origin_lat << std::endl;
    std::cout << "  origin_lon: " << origin_lon << std::endl;
    std::cout << "  system_id: " << system_id << std::endl;
    std::cout << std::endl;
  }
};

/**
 * @struct CheckpointData
 * @brief checkpoint一行分のデータを保持
 */
class PoseData
{
public:
  double x;
  double y;
  double z;
  int z_mode;
  double roll;
  double yaw;
  double wait_ms;
  bool fine;

  PoseData(
    const double x, const double y, const double z, const int z_mode, const double roll,
    const double yaw, const double wait_ms, const bool fine = false)
  : x(x), y(y), z(z), z_mode(z_mode), roll(roll), yaw(yaw), wait_ms(wait_ms), fine(fine)
  {
  }

  void print() const
  {
    std::cout << "  x:" << std::right << std::setw(7) << std::fixed << std::setprecision(2) << x;
    std::cout << "  y:" << std::right << std::setw(7) << std::fixed << std::setprecision(2) << y;
    std::cout << "  z:" << std::right << std::setw(7) << std::fixed << std::setprecision(2) << z;
    std::cout << "  z_mode:" << std::right << std::setw(2) << z_mode;
    std::cout << "  roll:" << std::right << std::setw(8) << std::fixed << std::setprecision(2)
              << roll;
    std::cout << "  yaw:" << std::right << std::setw(8) << std::fixed << std::setprecision(2)
              << yaw;
    std::cout << "  wait_ms:" << std::right << std::setw(8) << std::fixed << std::setprecision(0)
              << wait_ms;
    std::cout << "  fine:" << std::right << std::setw(6) << (fine ? "true" : "false");
    std::cout << std::endl;
  }
};

const size_t NUM_CSV_COLUMNS = 19;
const std::array<std::string, NUM_CSV_COLUMNS> csv_header = {
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
  "catmull_fine",
};

/**
 * @class CsvData
 * @brief パースしたCSV全体のデータを保持
 */
class PathData
{
private:
  std::array<std::string, NUM_CSV_COLUMNS> header = csv_header;
  ParamData params_;
  std::vector<PoseData> checkpoints_;
  std::vector<PoseData> catmulls_;

  // CsvParserからのみ内部データへのアクセスを許可
  friend class PathCsvLoader;

public:
  const ParamData & get_params() const { return params_; }
  const std::vector<PoseData> & get_checkpoints() const { return checkpoints_; }
  const std::vector<PoseData> & get_catmulls() const { return catmulls_; }
};

/**
 * @class PathCsvLoader
 * @brief load and parse csv file with path
 */
class PathCsvLoader
{
private:
  std::shared_ptr<PathData> data_;

  // helper
  double stod_strict(const std::string & s, const std::string & context);
  int stoi_strict(const std::string & s, const std::string & context);
  bool stob_strict(const std::string & s, const std::string & context);

  bool is_allNonEmpty(std::vector<std::string> vec);

public:
  explicit PathCsvLoader();

  /**
     * @brief CSVファイルをパースする
     * @param csv_path パース対象のCSVファイルパス
     */
  void parse(const std::string & csv_path);

  /**
     * @brief パースしたデータを取得する
     * @return 読み込んだCsvDataへのconst参照
     */
  const std::shared_ptr<PathData> get_data() const { return data_; }

  /**
     * @brief Set default origin for geodetic conversion
     * @param lat Origin latitude
     * @param lon Origin longitude
     * @param sys_id System ID (1-19)
     */
  void setDefaultOrigin(double lat, double lon, int sys_id)
  {
    data_->params_.origin_lat = lat;
    data_->params_.origin_lon = lon;
    data_->params_.system_id = sys_id;
  }
};

}  // namespace planner

#endif
