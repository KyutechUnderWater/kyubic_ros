#include "ament_index_cpp/get_package_share_directory.hpp"
#include "path_planner/path_csv_loader.hpp"

#include <filesystem>
#include <iostream>
#include <memory>

// main.cpp 内で path_planner 名前空間の要素を直接使えるようにする
using namespace planner;

int main()
{
  // 1. パッケージ名で共有ディレクトリのパスを取得
  std::string package_path;
  try {
    // "your_package_name" は実際のパッケージ名に置き換え
    package_path = ament_index_cpp::get_package_share_directory("path_planner");
  } catch (char * e) {
    // RCLCPP_ERRORやstd::cerrでエラーを出力
    std::cerr << "Package 'your_package_name' not found: " << std::endl;
    // エラーハンドリング (プログラム終了など)
    return 1;
  }

  // 2. ファイルへのフルパスを構築
  std::filesystem::path csv_file_path =
    std::filesystem::path(package_path) / "assets" / "test_generated.csv";

  // --- パース処理の実行 ---
  PathCsvLoader loader;
  try {
    loader.parse(csv_file_path);
    std::cout << "CSV parsing successful. ✅\n\n";

    // --- パース結果の表示 ---
    const std::shared_ptr<PathData> data = loader.get_data();

    std::cout << "--- Parameters ---\n";
    data->get_params().print();

    std::cout << "\n--- Checkpoints (" << data->get_checkpoints().size() << " items) ---\n";
    for (auto p : data->get_checkpoints()) {
      p.print();
    }

    std::cout << "\n--- Catmulls (" << data->get_catmulls().size() << " items) ---\n";
    for (auto p : data->get_catmulls()) {
      p.print();
    }

  } catch (const std::runtime_error & e) {
    std::cerr << "❌ PARSING FAILED: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
