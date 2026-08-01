#ifndef _WRITE_HYDROPHONE_WAYPOINT_CSV_HPP
#define _WRITE_HYDROPHONE_WAYPOINT_CSV_HPP

#include <behaviortree_cpp/action_node.h>

#include <localization_msgs/msg/odometry.hpp>
#include <planner_msgs/msg/pinger_direction.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief pinger_direction(機体座標系の相対yaw/pitch)とodomから、step_distance_m先の
 * 1点をpath_planner形式のCSVへ書き出すBT葉ノード(iwakuni2026.xml、ハイドロフォン
 * 追跡のステップ移動)。
 * @details odom/pinger_directionのどちらかを一度も受信していない間はRUNNINGで待機
 * (GoToDepth/CheckBuoyDetectedと同じ安全側の設計)。両方揃ったら、CheckBuoyDetectedと
 * 同じtf2::Quaternion::setRPY + quatRotateのパターンで機体座標系のオフセットを
 * NED絶対座標へ回転し、CSVを一時ファイル+renameで原子的に書き出してSUCCESSを返す
 * (1回書いたら完了。WaypointAction側の移動完了は待たない)。
 *
 * 【未検証】pinger_direction.pitchの正方向(上向きが正、という仮定で実装)は
 * sbl_estimation_node側の実装依存。実機で符号が逆だった場合、
 * write_hydrophone_waypoint_csv.cpp内のbody-frameオフセットのz成分の符号
 * (1箇所)を反転させれば直る設計にしてある。
 *
 * yaw: step_distance_m>0(通常のステップ移動)では現在のyawを維持する(向きは変えず
 * 位置だけ進める)。step_distance_m==0.0の場合のみ、その場での「向き直り」用途
 * (iwakuni2026.xmlのTurnTowardPinger)として、target_yawをピンガー方向
 * (current_yaw + pinger_direction.yaw)へ切り替える。
 *
 * csv_file_pathが'/'始まりでない場合、path_plannerパッケージのshareディレクトリを
 * 基準に解決する(pdla_planner.cppと同じ解決規則。WaypointActionが最終的に同じ
 * ファイルを読むため、解決規則を合わせる必要がある)。書き込み先ディレクトリが
 * 無ければ作成する。
 */
class WriteHydrophoneWaypointCSV : public BT::StatefulActionNode
{
public:
  WriteHydrophoneWaypointCSV(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<planner_msgs::msg::PingerDirection>::SharedPtr pinger_sub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr odom_sub_;
  planner_msgs::msg::PingerDirection::SharedPtr latest_pinger_;
  localization_msgs::msg::Odometry::SharedPtr latest_odom_;

  void pingerCallback(const planner_msgs::msg::PingerDirection::SharedPtr msg);
  void odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace bluerov_control_bt_nodes

#endif
