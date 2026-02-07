/**
 * @file wrench_plan_selector_component.hpp
 * @brief Declaration of the WrenchPlanSelector component.
 */

#ifndef WRENCH_PLAN_SELECTOR__WRENCH_PLAN_SELECTOR_COMPONENT_HPP_
#define WRENCH_PLAN_SELECTOR__WRENCH_PLAN_SELECTOR_COMPONENT_HPP_

#include "planner_msgs/msg/wrench_plan.hpp"
#include "rclcpp/rclcpp.hpp"
#if defined(_WIN32)
#if defined(PLANNER_BUILDING_DLL)
#define PLANNER_PUBLIC __declspec(dllexport)
#else
#define PLANNER_PUBLIC __declspec(dllimport)
#endif
#else
#define PLANNER_PUBLIC
#endif

namespace planner::wrench_planner
{

/**
 * @struct BufferedWrenchPlan
 * @brief Internal structure to hold the message and its arrival time.
 */
struct BufferedWrenchPlan
{
  planner_msgs::msg::WrenchPlan msg;
  rclcpp::Time arrival_time;
};

class WrenchPlanSelector : public rclcpp::Node
{
public:
  PLANNER_PUBLIC
  explicit WrenchPlanSelector(const rclcpp::NodeOptions & options);

private:
  /**
     * @brief Callback executed when new data arrives.
     * @details Updates buffer and publishes immediately if it's the highest priority.
     */
  void topic_callback(const planner_msgs::msg::WrenchPlan::SharedPtr msg);

  /**
     * @brief Periodic timer (50ms) to clean up timeouts and handle priority transitions.
     */
  void timer_callback();

  /**
     * @brief Helper to check if a buffered plan is expired.
     */
  bool is_expired(const BufferedWrenchPlan & plan, const rclcpp::Time & now) const;

  rclcpp::Subscription<planner_msgs::msg::WrenchPlan>::SharedPtr subscription_;
  rclcpp::Publisher<planner_msgs::msg::WrenchPlan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Buffer: Key = Priority ID (Ascending order), Value = Message Data
  std::map<uint8_t, BufferedWrenchPlan> active_plans_;

  // Tracks the ID of the plan that was most recently published/active.
  // Used to detect transitions (e.g., ID 1 timed out -> ID 10 takes over).
  std::optional<uint8_t> last_published_id_;
};

}  // namespace planner::wrench_planner

#endif  // WRENCH_PLAN_SELECTOR__WRENCH_PLAN_SELECTOR_COMPONENT_HPP_
