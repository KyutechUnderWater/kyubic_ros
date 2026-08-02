#ifndef _WRITE_VISION_WAYPOINT_CSV_HPP
#define _WRITE_VISION_WAYPOINT_CSV_HPP

#include <behaviortree_cpp/action_node.h>

#include <localization_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief perception/buoy_detector(buoy_detector_node.py)が書き出す固定CSVの最終行と
 * odomから、ブイを画面中心へ寄せながら前進する1点をpath_planner形式のCSVへ書き出すBT葉
 * ノード(iwakuni2026.xml、`CenteringStep`)。`WriteHydrophoneWaypointCSV`と構造は同じ
 * (odom/検出を待つ→計算→waypoint_csv_writerでatomic write)だが、入力元と、水平/垂直で
 * 別の情報源から補正量を作る点が異なる。
 * @details 以前は`/buoy/relative_position`(buoy_interfaces/msg/BuoyRelativePosition)を
 * 直接購読していたが、buoy_detector_node.pyが同じ検出値を毎フレーム追記している固定CSVを
 * `buoy_detection_csv_reader`経由で読み取る方式に変更した(パス・列レイアウトは
 * buoy_detection_csv_reader.hpp参照)。
 *
 * 計算の流れ:
 * 1. CSVが未生成・stale_timeout_sec超過・パース不能ならRUNNING(安全側で待機)。
 *    confidenceが`confidence_threshold`未満、または`detected==false`ならこのtickは
 *    何もせずFAILURE(RetryUntilSuccessfulが次tickで再試行する。安全側)。
 * 2. yaw補正: `yaw_valid`なら`yaw_rad`をそのまま使う。そうでなければ
 *    `normalized_horizontal_offset × (horizontal_fov_rad / 2)`で近似する
 *    (`horizontal_fov_rad`未指定かつ`yaw_valid=false`の場合はFAILURE)。
 * 3. `|yaw補正| < deadband_rad`ならyaw補正を0にする(ジッター防止の不感帯)。
 * 4. 新しいyaw[rad] = 現在yaw[rad] + yaw補正[rad](odomのyawは度単位のため変換が必要)。
 * 5. z_step: `position_valid`なら`z_m × vertical_gain_m`
 *    (【未検証・要実機確認】`z_m`は機体座標系で正=下方向、NED慣習と一致する前提。
 *    `vertical_gain_m`はここでは無次元の比例ゲインとして使う("1tickでこの割合だけ
 *    縦方向の推定誤差に近づく"という意味。仕様書は元々`normalized_vertical_offset`
 *    (無次元、画像処理班が未実装)を使う想定だったが、そのフィールドが
 *    `BuoyRelativePosition.msg`に存在しないため、依頼者と合意の上`z_m`で代用した)。
 *    `position_valid=false`の間はz_step=0(垂直方向の補正はせず、水平方向の中心合わせ
 *    のみ進める)。
 * 6. target_x/y = 現在x/y + step_distance_m × cos/sin(新しいyaw[rad])。
 *    target_z = 現在z(depth基準) + z_step。target_yaw = 新しいyaw(度へ変換)。
 *    rollは常に現在値を維持。
 * 7. waypoint_csv_writerで19列フォーマットのCSVへatomic write。
 *
 * odomを一度も受信していない間はRUNNINGのまま待機する(WriteHydrophoneWaypointCSV等と
 * 同じ安全側の設計)。
 */
class WriteVisionWaypointCSV : public BT::StatefulActionNode
{
public:
  WriteVisionWaypointCSV(
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
