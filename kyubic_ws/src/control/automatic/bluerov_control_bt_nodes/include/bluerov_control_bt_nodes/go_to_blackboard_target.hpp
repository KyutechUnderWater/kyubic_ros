#ifndef _GO_TO_BLACKBOARD_TARGET_HPP
#define _GO_TO_BLACKBOARD_TARGET_HPP

#include <behaviortree_cpp/action_node.h>

#include <localization_msgs/msg/odometry.hpp>
#include <planner_msgs/msg/wrench_plan.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bluerov_control_bt_nodes
{

/**
 * @brief BTブラックボードに書き込まれた絶対座標(x, y, 深度z)へ向かうBT葉ノード。
 * @details CheckPingerFound/CheckBuoyDetectedが discovered_target_x/y/z へ書き込んだ値を
 * XML側で {discovered_target_x} のようにポートへバインドして渡す想定
 * (旧bluerov_control/flow.pyの VISUAL_ATTACK / HYDRO_APPROACH 相当)。
 * roll/yawはGoToDepthと同様、起動時点のodom値を維持する(wrench_plannerはworld座標系で
 * x/y/yawを独立制御できるため、ホバリング型AUVでは目標方向を向く必要はない)。
 * odom未受信、またはtarget_x/y/zポートが未指定の間はRUNNINGのまま安全側で待機する。
 *
 * フィールド境界チェック: mission_origin_x/y(CaptureMissionOriginがブラックボードへ
 * 書き込んだ原点)から+x方向・+y方向へfield_size_mを超えるtarget_x/yは誤検出とみなし、
 * publishせずFAILUREを返す(非対称な矩形。+x/+yがフィールド奥方向を向くことは、
 * 機体を投入地点でフィールド奥を向けて起動する、という運用手順で担保する)。
 * mission_origin_x/yポートが未設定(CaptureMissionOrigin未実行等)の場合は安全側で
 * RUNNINGのまま待機し、境界チェックを省略してpublishすることはない。
 */
class GoToBlackboardTarget : public BT::StatefulActionNode
{
public:
  GoToBlackboardTarget(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<planner_msgs::msg::WrenchPlan>::SharedPtr publisher_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr odom_sub_;
  localization_msgs::msg::Odometry::SharedPtr latest_odom_;

  bool published_{false};
  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};

  void odomCallback(const localization_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace bluerov_control_bt_nodes

#endif
