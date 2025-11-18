#include <chrono>
#include <iostream>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/control_node.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

// ROS 2 パッケージのshareディレクトリからファイルパスを取得するために必要
#include "ament_index_cpp/get_package_share_directory.hpp"

/* --- カスタムActionノードの定義 --- */

// case_1 ('TaskA') で実行されるアクション
class ApproachObject : public BT::StatefulActionNode
{
public:
  ApproachObject(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config), counter_(0)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  // ノードが最初にティックされたときに1回だけ呼び出される
  BT::NodeStatus onStart() override
  {
    std::cout << ">>> [TaskA] ApproachObject: 開始 (フェーズ: " << counter_ << ")" << std::endl;
    counter_ = 0;                    // カウンターをリセット
    return BT::NodeStatus::RUNNING;  // 処理はまだ終わっていないので RUNNING を返す
  }

  // ノードが RUNNING を返している間、毎ティック呼び出される
  BT::NodeStatus onRunning() override
  {
    counter_++;
    std::cout << ">>> [TaskA] ApproachObject: 実行中... (フェーズ: " << counter_ << ")"
              << std::endl;

    // 3回RUNNINGを返したらSUCCESSを返すようにする
    if (counter_ >= 3) {
      std::cout << ">>> [TaskA] ApproachObject: 完了！" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;  // まだ処理中なので RUNNING を返す
  }

  // ノードが中断されたときに呼び出される
  void onHalted() override
  {
    std::cout << ">>> [TaskA] ApproachObject: 中断されました。" << std::endl;
    counter_ = 0;  // 必要に応じて状態をリセット
  }

private:
  int counter_;
};

// case_2 ('TaskB') で実行されるアクション
class GraspObject : public BT::SyncActionNode
{
public:
  GraspObject(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    std::cout << ">>> [TaskB] 実行: GraspObject" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

// デフォルトケースで実行されるアクション
class Patrol : public BT::SyncActionNode
{
public:
  Patrol(const std::string & name, const BT::NodeConfig & config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    std::cout << ">>> [Default] 実行: Patrol" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

/* --- main関数 --- */

int main()
{
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<BT::SwitchNode<2>>("Switch");

  // カスタムノードをファクトリに登録
  factory.registerNodeType<ApproachObject>("ApproachObject");
  factory.registerNodeType<GraspObject>("GraspObject");
  factory.registerNodeType<Patrol>("Patrol");

  // --- ROS 2 パッケージのパスからXMLを読み込む ---
  std::string package_path;
  try {
    package_path = ament_index_cpp::get_package_share_directory("bt_switch_example");
  } catch (const std::exception & e) {
    std::cerr << "パッケージ 'bt_switch_example' が見つかりません: " << e.what() << std::endl;
    return 1;
  }

  const std::string xml_file_path = package_path + "/bt_xml/switch_example.xml";
  std::cout << "読み込むXMLファイル: " << xml_file_path << std::endl;

  try {
    // XMLファイルからツリー定義を登録
    factory.registerBehaviorTreeFromFile(xml_file_path);
  } catch (const BT::RuntimeError & e) {
    std::cerr << "エラー: XMLファイルの読み込みに失敗。 " << e.what() << std::endl;
    return 1;
  }

  // 登録したMainTreeからインスタンスを作成
  auto tree = factory.createTree("MainTree");

  // シンプルなコンソールロガーを追加
  BT::StdCoutLogger logger(tree);

  std::cout << "--- Switchの動作テスト開始 ---" << std::endl;

  // MainTreeのSequenceが完了するまで実行
  tree.tickWhileRunning(std::chrono::milliseconds(100));

  std::cout << "--- テスト終了 ---" << std::endl;

  return 0;
}
