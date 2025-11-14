#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "rclcpp/rclcpp.hpp"

// カスタムBTノードのヘッダー
#include "robot_bt_controller/check_emergency_state.hpp"
#include "robot_bt_controller/check_joy_button.hpp"
#include "robot_bt_controller/lifecycle_manager.hpp"

class BtManagerNode : public rclcpp::Node
{
public:
  explicit BtManagerNode() : Node("bt_manager_node")
  {
    // BTファクトリの初期化
    factory_.reset(new BT::BehaviorTreeFactory());

    // 1. カスタムBTノードをファクトリに登録
    //    カスタムノードがROSの機能(Sub/Pub/Service)を使えるよう、
    //    このマネージャノードのポインタをBlackboard経由で渡す。
    BT::NodeConfiguration config;
    config.blackboard = BT::Blackboard::create();
    config.blackboard->set<rclcpp::Node::SharedPtr>("node", this);  // "node"というキーで保存

    // ノードを登録
    factory_->registerNodeType<CheckEmergencyState>("CheckEmergencyState", config);
    factory_->registerNodeType<CheckJoyButton>("CheckJoyButton", config);
    factory_->registerNodeType<LifecycleManager>("LifecycleManager", config);

    // 2. BT XMLファイルのパスを取得
    std::string pkg_share = ament_index_cpp::get_package_share_directory("robot_bt_controller");
    std::string xml_file = pkg_share + "/bt_xml/robot_control_tree.xml";
    RCLCPP_INFO(get_logger(), "Loading BT XML: %s", xml_file.c_str());

    // 3. ツリーの作成
    tree_ = factory_->createTreeFromFile(xml_file);

    // (オプション) 標準出力ロガー
    auto logger = std::make_unique<BT::StdCoutLogger>(tree_);

    // 4. ツリーを10HzでTICKするタイマー
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 10Hz
      [this]() {
        // ツリーを実行
        tree_.tickOnce();
      });
  }

private:
  std::unique_ptr<BT::BehaviorTreeFactory> factory_;
  BT::Tree tree_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BtManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}