/**
 * @file path_csv_parser.cpp
 * @brief Parse csv file with route infomation
 * @author R.Ohnishi
 * @date 2025/07/19
 *
 * @details 経路情報を持つcsvファイルの構文解析
 **********************************************/

#include "path_planner/path_csv_loader.hpp"

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace planner
{

PathCsvLoader::PathCsvLoader() { data_ = std::make_shared<PathData>(); }

double PathCsvLoader::stod_strict(const std::string & s, const std::string & context)
{
  if (s.empty()) {
    throw std::runtime_error("Empty value not allowed for " + context);
  }
  try {
    return std::stod(s);
  } catch (...) {
    throw std::runtime_error("Cannot convert '" + s + "' to double for " + context);
  }
}

int PathCsvLoader::stoi_strict(const std::string & s, const std::string & context)
{
  if (s.empty()) {
    throw std::runtime_error("Empty value not allowed for " + context);
  }
  try {
    return static_cast<int>(std::stod(s));
  } catch (...) {
    throw std::runtime_error("Cannot convert '" + s + "' to int for " + context);
  }
}

bool PathCsvLoader::stob_strict(const std::string & s, const std::string & context)
{
  if (s.empty()) {
    throw std::runtime_error("Empty value not allowed for " + context);
  }
  try {
    return static_cast<bool>(std::stod(s));
  } catch (...) {
    throw std::runtime_error("Cannot convert '" + s + "' to int for " + context);
  }
}

bool PathCsvLoader::is_allNonEmpty(std::vector<std::string> vec)
{
  return std::all_of(vec.begin(), vec.end(), [](const std::string & s) { return !s.empty(); });
}

void PathCsvLoader::parse(const std::string & csv_path)
{
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    throw std::runtime_error("Error: Could not open file: " + csv_path);
  }

  // --- ファイル全体をメモリに読み込む ---
  std::vector<std::vector<std::string>> raw_data;
  std::string line;

  // ヘッダー行の確認
  std::getline(file, line);
  std::stringstream ss(line);
  std::string token;
  std::vector<std::string> raw_header;
  while (std::getline(ss, token, ',')) {
    raw_header.push_back(token);
  }

  // ヘッダーの列数と内容を比較
  if (raw_header.size() != NUM_CSV_COLUMNS) {
    throw std::runtime_error(
      "Invalid CSV header: Column count is " + std::to_string(raw_header.size()) + ", expected " +
      std::to_string(NUM_CSV_COLUMNS) + ".");
  }
  for (size_t i = 0; i < NUM_CSV_COLUMNS; ++i) {
    if (raw_header[i] != csv_header[i]) {
      throw std::runtime_error(
        "Invalid CSV header at column " + std::to_string(i + 1) + ". Expected '" +
        csv_header[i].c_str() + "' but got '" + raw_header[i].c_str() + "'.");
    }
  }

  int line_num = 1;
  while (std::getline(file, line)) {
    line_num++;

    std::stringstream ss(line);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(ss, token, ',')) {
      tokens.push_back(token);
    }

    // データがあるか確認
    if (tokens.size() != NUM_CSV_COLUMNS) {
      throw std::runtime_error(
        "Invalid data row at source line " + std::to_string(line_num) + ": Column count is " +
        std::to_string(tokens.size()) + ", but expected " + std::to_string(NUM_CSV_COLUMNS) + ".");
    }
    raw_data.push_back(tokens);
  }

  // --- 第1パス: パラメータ抽出 ---
  for (const auto & tokens : raw_data) {
    if (!tokens[0].empty()) {  // parameter_label列に値がある行
      const std::string & label = tokens[0];
      if (label == "checkpoint_end_row") {
        data_->params_.checkpoint_end_row = stoi_strict(tokens[1], label);
      } else if (label == "catmull_rom") {
        data_->params_.catmull_rom = stoi_strict(tokens[1], label);
      } else if (label == "catmull_end_row") {
        data_->params_.catmull_end_row = stoi_strict(tokens[1], label);
      } else if (label == "catmull_density") {
        data_->params_.catmull_density = stoi_strict(tokens[1], label);
      } else if (label == "catmull_min_distance") {
        data_->params_.catmull_min_distance = stod_strict(tokens[1], label);
      } else if (label == "catmull_orient_LERP") {
        data_->params_.catmull_orient_LERP = stoi_strict(tokens[1], label);
      } else if (label == "timeout_sec") {
        data_->params_.timeout_sec = stoi_strict(tokens[1], label);
      } else {
        throw std::runtime_error("Unkown parameter " + label);
      }
    }
  }

  // --- 第2パス: データ行を解析 ---
  for (size_t i = 0; i < raw_data.size(); ++i) {
    auto & tokens = raw_data[i];
    try {
      // Checkpointデータ
      if (is_allNonEmpty(std::vector<std::string>(tokens.begin() + 2, tokens.begin() + 7))) {
        data_->checkpoints_.push_back(PoseData(
          stod_strict(tokens[2], "x"), stod_strict(tokens[3], "y"), stod_strict(tokens[4], "z"),
          stoi_strict(tokens[5], "z_mode"), stod_strict(tokens[6], "roll"),
          stod_strict(tokens[7], "yaw"), stod_strict(tokens[8], "wait_ms"),
          stob_strict(tokens[9], "fine")));
      }
      // Catmullデータ
      uint8_t offset = 9;
      if (is_allNonEmpty(
            std::vector<std::string>(tokens.begin() + offset + 2, tokens.begin() + offset + 9))) {
        data_->catmulls_.push_back(PoseData(
          stod_strict(tokens[offset + 2], "catmull_x"),
          stod_strict(tokens[offset + 3], "catmull_y"),
          stod_strict(tokens[offset + 4], "catmull_z"),
          stoi_strict(tokens[offset + 5], "catmull_z_mode"),
          stod_strict(tokens[offset + 6], "catmull_roll"),
          stod_strict(tokens[offset + 7], "catmull_yaw"),
          stod_strict(tokens[offset + 8], "catmull_wait_ms"),
          stob_strict(tokens[offset + 9], "catmull_fine")));
      }
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Error parsing data at source line " + std::to_string(i + 2) + ": " + e.what());
    }
  }

  // --- 抽出されたデータ数と宣言された数が一致するか ---
  if (data_->checkpoints_.size() != data_->params_.checkpoint_end_row) {
    throw std::runtime_error(
      "Checkpoint count mismatch. Expected: " + std::to_string(data_->params_.checkpoint_end_row) +
      ", Found: " + std::to_string(data_->checkpoints_.size()));
  }
  if (data_->catmulls_.size() != data_->params_.catmull_end_row) {
    throw std::runtime_error(
      "Catmull count mismatch. Expected: " + std::to_string(data_->params_.catmull_end_row) +
      ", Found: " + std::to_string(data_->catmulls_.size()));
  }
}

}  // namespace planner
