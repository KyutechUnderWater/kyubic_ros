#ifndef _WRITE_AXIS_OVERRIDE_WAYPOINT_CSV_HPP
#define _WRITE_AXIS_OVERRIDE_WAYPOINT_CSV_HPP

#include <behaviortree_cpp/action_node.h>

#include <localization_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief 現在のodomを基準に、指定した軸(x/y/z/yaw)だけ上書きした1点をpath_planner形式の
 * CSVへ書き出すBT葉ノード(iwakuni2026.xml)。
 * @details iwakuni2026.xmlの設計方針: `GoToDepth`のような自前でWrenchPlanを出し続ける
 * 独自移動ノードは使わず、「現在odom基準の1点CSVを書いて既存のWaypointAction(PDLA、
 * 実績のある経路)へ処理を任せる」方式に統一する。このノードはその「軸ごとの部分上書き」を
 * 担う汎用版で、`GoToDepth`(深度/高度専用)を置き換える。
 *
 * `override_x`/`override_y`/`override_z`/`override_yaw`のうち指定されたポートだけその値を
 * 使い、指定されなかった軸は起動時点のodomの値をそのまま使う(部分上書き)。rollはこのノードの
 * 上書き対象ではなく常に現在値を使う。
 *
 * z軸の扱い: `z_mode`("depth"(既定)/"altitude")で選んだ基準に応じて、`override_z`が
 * 未指定のときのフォールバック値も`z_altitude`/`z_depth`のどちらかに切り替わる(GoToDepthの
 * z_modeポートと同じ考え方。CSVのz_mode列が1列しか無いため、値の基準を混在させないため)。
 *
 * odomを一度も受信していない間はRUNNINGのまま待機する(GoToDepth/CaptureMissionOriginと
 * 同じ安全側の設計)。書き込みが成功したら1回でSUCCESSを返す(実際の移動・完了判定は
 * 後続のWaypointActionが担う。このノード自体はodomを待つ以外、動きの完了は待たない)。
 *
 * CSV書き込み(19列フォーマット・パス解決・atomic write)は`waypoint_csv_writer`
 * (`WriteHydrophoneWaypointCSV`と共用)を使う。
 */
class WriteAxisOverrideWaypointCSV : public BT::StatefulActionNode
{
public:
  WriteAxisOverrideWaypointCSV(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr odom_sub_;
  localization_msgs::msg::Odometry::SharedPtr latest_odom_;

  void odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace bluerov_control_bt_nodes

#endif
