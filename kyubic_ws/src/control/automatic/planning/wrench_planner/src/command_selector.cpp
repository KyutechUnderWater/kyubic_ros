/**
 * @file wrench_plan_selector_component.cpp
 * @brief Implementation of the WrenchPlanSelector component.
 */

#include "wrench_planner/command_selector.hpp"

using namespace std::chrono_literals;

namespace planner::wrench_planner
{

WrenchPlanSelector::WrenchPlanSelector(const rclcpp::NodeOptions & options)
: Node("wrench_plan_selector", options)
{
  rclcpp::QoS qos(10);

  // Initialize Subscriber
  subscription_ = this->create_subscription<planner_msgs::msg::WrenchPlan>(
    "wrench_plan", qos,
    std::bind(&WrenchPlanSelector::topic_callback, this, std::placeholders::_1));

  // Initialize Publisher
  publisher_ = this->create_publisher<planner_msgs::msg::WrenchPlan>("selected_wrench_plan", qos);

  // Initialize Timer (50ms) for timeout checks and transitions
  timer_ = this->create_wall_timer(50ms, std::bind(&WrenchPlanSelector::timer_callback, this));

  RCLCPP_INFO(
    this->get_logger(), "WrenchPlan Selector started in namespace planner::wrench_planner");
}

bool WrenchPlanSelector::is_expired(const BufferedWrenchPlan & plan, const rclcpp::Time & now) const
{
  double timeout_sec = plan.msg.priority.timeout_ms / 1000.0;
  if (timeout_sec == 0.0) {
    return false;
  }

  double elapsed_sec = (now - plan.arrival_time).seconds();
  return elapsed_sec > timeout_sec;
}

void WrenchPlanSelector::topic_callback(const planner_msgs::msg::WrenchPlan::SharedPtr msg)
{
  rclcpp::Time current_time = this->now();

  // 1. Always store/update the incoming message in the buffer
  BufferedWrenchPlan new_entry;
  new_entry.msg = *msg;
  new_entry.arrival_time = current_time;
  active_plans_[msg->priority.id] = new_entry;

  // 2. Determine if this new message should be published immediately.
  // It should be published if it is the "Highest Priority VALID message".

  // Find the highest priority message that hasn't expired yet
  uint8_t best_valid_id = 255;  // Max uint8
  bool found_valid = false;

  for (const auto & [id, plan] : active_plans_) {
    if (!is_expired(plan, current_time)) {
      best_valid_id = id;
      found_valid = true;
      break;  // Since map is sorted, the first valid one is the best
    }
  }

  // 3. Publish if the incoming message corresponds to the best valid ID
  if (found_valid && best_valid_id == msg->priority.id) {
    publisher_->publish(*msg);
    last_published_id_ = msg->priority.id;

    // Debug
    RCLCPP_INFO(this->get_logger(), "Immediate Publish ID: %d", msg->priority.id);
  }
}

void WrenchPlanSelector::timer_callback()
{
  rclcpp::Time current_time = this->now();

  // 1. Clean up expired messages
  bool cleanup_occurred = false;
  for (auto it = active_plans_.begin(); it != active_plans_.end();) {
    if (is_expired(it->second, current_time)) {
      RCLCPP_INFO(this->get_logger(), "ID %d timed out", it->first);
      it = active_plans_.erase(it);
      cleanup_occurred = true;
    } else {
      ++it;
    }
  }

  // 2. Handle transitions
  // If we cleaned up (e.g., high priority timed out), we need to check
  // if a lower priority message should now take over.

  if (active_plans_.empty()) {
    // No valid plans left. Reset state.
    last_published_id_.reset();
    return;
  }

  // The map is sorted, so the first element is the highest priority
  const auto & best_entry = *active_plans_.begin();
  uint8_t current_best_id = best_entry.first;

  // Logic:
  // If the "current best ID" is different from what we last published,
  // it means a higher priority message timed out and we fell back to this one.
  // We should publish this "new best" message to notify the system of the switch.

  bool should_publish = false;

  if (!last_published_id_.has_value()) {
    // First time publishing or after complete reset
    should_publish = true;
  } else if (last_published_id_.value() != current_best_id) {
    // ID changed (Transition occurred)
    should_publish = true;
  }

  if (should_publish) {
    publisher_->publish(best_entry.second.msg);
    last_published_id_ = current_best_id;

    RCLCPP_INFO(this->get_logger(), "Timeout Transition -> Switched to ID: %d", current_best_id);
  }
}

}  // namespace planner::wrench_planner

// Register the component with the ROS 2 class loader
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planner::wrench_planner::WrenchPlanSelector)
