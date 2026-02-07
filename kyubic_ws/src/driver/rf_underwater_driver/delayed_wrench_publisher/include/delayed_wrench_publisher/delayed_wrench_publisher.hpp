/**
 * @file delayed_wrench_publisher.hpp
 * @brief Header definition for the DelayedWrenchPublisher node.
 */

#ifndef DELAYED_WRENCH_PUBLISHER_HPP_
#define DELAYED_WRENCH_PUBLISHER_HPP_

#include "planner_msgs/msg/pdla_feedback.hpp"
#include "planner_msgs/msg/wrench_plan.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @class DelayedWrenchPublisher
 * @brief Node that triggers a WrenchPlan publication sequence upon receiving the first feedback.
 */
class DelayedWrenchPublisher : public rclcpp::Node
{
public:
  /**
   * @brief Constructor.
   */
  DelayedWrenchPublisher();

private:
  /**
   * @brief Populates the WrenchPlan message with predefined values.
   * @note Hardcoded data is defined here.
   */
  void init_wrench_plan_data();

  /**
   * @brief Callback function for PDLAFeedback.
   * Triggers the sequence only once.
   * @param msg The received feedback message.
   */
  void feedback_callback(const planner_msgs::msg::PDLAFeedback::SharedPtr msg);

  /**
   * @brief Callback function executed after the 10-second delay.
   * Starts the periodic publication.
   */
  void delay_timer_callback();

  /**
   * @brief Callback function executed at 5Hz.
   * Publishes the WrenchPlan message.
   */
  void publish_timer_callback();

  // --- Member Variables ---

  //! Subscription for feedback
  rclcpp::Subscription<planner_msgs::msg::PDLAFeedback>::SharedPtr subscription_;

  //! Publisher for wrench plan
  rclcpp::Publisher<planner_msgs::msg::WrenchPlan>::SharedPtr publisher_;

  //! One-shot timer for the initial delay
  rclcpp::TimerBase::SharedPtr delay_timer_;

  //! Periodic timer for publishing data
  rclcpp::TimerBase::SharedPtr publish_timer_;

  //! The message object to be published
  planner_msgs::msg::WrenchPlan wrench_msg_;

  //! Flag to ensure the trigger happens only once
  bool is_triggered_;
};

#endif  // DELAYED_WRENCH_PUBLISHER_HPP_
