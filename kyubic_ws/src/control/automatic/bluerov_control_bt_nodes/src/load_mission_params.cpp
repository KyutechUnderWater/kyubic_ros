#include "bluerov_control_bt_nodes/load_mission_params.hpp"

#include <cmath>

namespace bluerov_control_bt_nodes
{

LoadMissionParams::LoadMissionParams(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::SyncActionNode(name, config), ros_node_(ros_node)
{
}

BT::PortsList LoadMissionParams::providedPorts() { return {}; }

double LoadMissionParams::declare_or_get_double(
  const std::string & name, const double default_value)
{
  if (!ros_node_->has_parameter(name)) {
    ros_node_->declare_parameter<double>(name, default_value);
  }
  return ros_node_->get_parameter(name).as_double();
}

BT::NodeStatus LoadMissionParams::tick()
{
  // config/bluerov_mission.param.yaml と同じキー名。単位はコメント参照。
  const double init_wait_sec = declare_or_get_double("init_wait_sec", 180.0);
  const double configure_delay_sec = declare_or_get_double("configure_delay_sec", 2.0);
  const double search_timeout_sec = declare_or_get_double("search_timeout_sec", 420.0);
  const double mic_confirm_timeout_sec = declare_or_get_double("mic_confirm_timeout_sec", 45.0);
  const double overall_mission_timeout_sec =
    declare_or_get_double("overall_mission_timeout_sec", 500.0);
  const double origin_capture_timeout_sec =
    declare_or_get_double("origin_capture_timeout_sec", 60.0);
  const double descend_depth_m = declare_or_get_double("descend_depth_m", 1.0);
  const double dive_offset_m = declare_or_get_double("dive_offset_m", 2.5);
  const double post_check_rise_offset_m = declare_or_get_double("post_check_rise_offset_m", 0.5);
  const double surface_rise_offset_m = declare_or_get_double("surface_rise_offset_m", 4.0);
  const double field_size_m = declare_or_get_double("field_size_m", 10.0);

  // GoToDepth がポート省略時に参照する ROS パラメータ(launch の depth_target_m で上書き済みの場合はその値)
  if (!ros_node_->has_parameter("default_depth_m")) {
    ros_node_->declare_parameter<double>("default_depth_m", descend_depth_m);
  }

  const int init_wait_ms = static_cast<int>(std::lround(init_wait_sec * 1000.0));
  const int configure_delay_ms = static_cast<int>(std::lround(configure_delay_sec * 1000.0));
  const int search_timeout_ms = static_cast<int>(std::lround(search_timeout_sec * 1000.0));
  const int mic_confirm_timeout_ms =
    static_cast<int>(std::lround(mic_confirm_timeout_sec * 1000.0));
  const int overall_mission_timeout_ms =
    static_cast<int>(std::lround(overall_mission_timeout_sec * 1000.0));
  const int origin_capture_timeout_ms =
    static_cast<int>(std::lround(origin_capture_timeout_sec * 1000.0));
  // GoToDepth depth_offset_m: 浮上は負(NED 正=下)
  const double post_check_depth_offset_m = -post_check_rise_offset_m;
  const double surface_depth_offset_m = -surface_rise_offset_m;

  if (overall_mission_timeout_sec <= search_timeout_sec) {
    RCLCPP_WARN(
      ros_node_->get_logger(),
      "LoadMissionParams: overall_mission_timeout_sec(%.0f)がsearch_timeout_sec(%.0f)以下です。"
      "Auto内側のCheckPingerFoundが自前の期限でSURFACEへ落ちる前に、外側Timeoutが"
      "AutoSequence全体をFAILUREにしてしまう可能性があります(config/bluerov_mission.param.yaml参照)",
      overall_mission_timeout_sec, search_timeout_sec);
  }

  // XML の {key} プレースホルダが参照するブラックボードキー
  config().blackboard->set("init_wait_ms", init_wait_ms);
  config().blackboard->set("configure_delay_ms", configure_delay_ms);
  config().blackboard->set("search_timeout_ms", search_timeout_ms);
  config().blackboard->set("mic_confirm_timeout_ms", mic_confirm_timeout_ms);
  config().blackboard->set("overall_mission_timeout_ms", overall_mission_timeout_ms);
  config().blackboard->set("origin_capture_timeout_ms", origin_capture_timeout_ms);
  config().blackboard->set("dive_offset_m", dive_offset_m);
  config().blackboard->set("post_check_depth_offset_m", post_check_depth_offset_m);
  config().blackboard->set("surface_depth_offset_m", surface_depth_offset_m);
  config().blackboard->set("field_size_m", field_size_m);

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "LoadMissionParams: init_wait=%.0fs configure_delay=%.1fs search_timeout=%.0fs mic_confirm=%.0fs "
    "overall_mission_timeout=%.0fs origin_capture_timeout=%.0fs "
    "descend_depth=%.2fm dive_offset=%.2fm post_check_rise=%.2fm surface_rise=%.2fm field_size=%.1fm",
    init_wait_sec, configure_delay_sec, search_timeout_sec, mic_confirm_timeout_sec,
    overall_mission_timeout_sec, origin_capture_timeout_sec,
    ros_node_->get_parameter("default_depth_m").as_double(), dive_offset_m,
    post_check_rise_offset_m, surface_rise_offset_m, field_size_m);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bluerov_control_bt_nodes
