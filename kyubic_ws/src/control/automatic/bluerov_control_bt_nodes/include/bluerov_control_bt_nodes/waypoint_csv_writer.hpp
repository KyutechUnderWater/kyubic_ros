#ifndef _WAYPOINT_CSV_WRITER_HPP
#define _WAYPOINT_CSV_WRITER_HPP

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace bluerov_control_bt_nodes
{

/**
 * @brief path_planner(PathCsvLoader)向けのCSV書き込み処理をまとめた共通ヘルパー。
 * @details 「現在odomを基準に1点のcheckpointを書き、実際の移動はWaypointAction(PDLA)に
 * 任せる」という設計のBT葉ノード(WriteHydrophoneWaypointCSV、WriteAxisOverrideWaypointCSV等)
 * から共通で使う。このヘルパー自体はBTノードではないため、bt_executor.cppへの登録は無い。
 */
namespace waypoint_csv_writer
{

/**
 * @brief csv_file_pathを絶対パスへ解決する。
 * @details '/'始まりならそのまま、そうでなければpath_plannerパッケージのshareディレクトリを
 * 基準に結合する(pdla_planner.cppと同じ解決規則)。WaypointActionが最終的に同じファイルを
 * 読むため、解決規則を合わせる必要がある。path_plannerの解決自体に失敗した場合はloggerへ
 * エラーを出し、csv_file_pathをそのまま返す(呼び出し元・WaypointAction側の双方が同じように
 * 解決に失敗する状況になるだけで、片方だけ違うパスを使ってしまう事態は避けられる)。
 */
std::string ResolvePath(const rclcpp::Logger & logger, const std::string & csv_file_path);

/**
 * @brief PathCsvLoaderが要求する19列固定・ヘッダー完全一致フォーマットで、ちょうど1点の
 * checkpointを一時ファイル+renameで原子的に書き出す。
 * @details 書き込み先ディレクトリが無ければ作成する。同一ディレクトリ内でrenameすることで、
 * WaypointAction側が書き込み途中の不完全なファイルを読んでしまうレースコンディションを防ぐ。
 * wait_ms/fineはこの用途(1点だけ経路を差し替えて即WaypointActionへ渡す)では常に0/falseで
 * 十分なため固定。
 * @throws std::runtime_error 一時ファイルの作成・書き込み・renameに失敗した場合。
 */
void WriteSingleCheckpoint(
  const std::string & resolved_path, double x, double y, double z, uint8_t z_mode, double roll,
  double yaw);

}  // namespace waypoint_csv_writer

}  // namespace bluerov_control_bt_nodes

#endif
