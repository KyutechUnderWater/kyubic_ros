#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace robot_controller
{
// rclcpp_lifecycle::LifecycleNode を継承
class ManualNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ManualNode(const rclcpp::NodeOptions & options) : LifecycleNode("manualNode", options)
  {
    RCLCPP_INFO(get_logger(), "ManualNode is created.");
  }

  // 1. Configure (設定)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Configuring ManualNode...");
    // 例: cmd_vel の Publisher などをここで初期化
    // cmd_vel_pub_ = this->create_publisher<...>(...);
    RCLCPP_INFO(get_logger(), "ManualNode configured.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 2. Activate (起動)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Activating ManualNode...");
    // 例: Joy の Subscriber をここで有効化 (またはタイマーを開始)
    // sub_ = this->create_subscription<...>(...);
    RCLCPP_INFO(get_logger(), "ManualNode activated.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 3. Deactivate (停止)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Deactivating ManualNode...");
    // 例: Subscriber をリセット (停止)
    // sub_.reset();
    RCLCPP_INFO(get_logger(), "ManualNode deactivated.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 4. Cleanup (クリーンアップ)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up ManualNode...");
    // Publisher などを破棄
    // cmd_vel_pub_.reset();
    RCLCPP_INFO(get_logger(), "ManualNode cleaned up.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 5. Shutdown (シャットダウン)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Shutting down ManualNode...");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
};
}  // namespace robot_controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_controller::ManualNode)