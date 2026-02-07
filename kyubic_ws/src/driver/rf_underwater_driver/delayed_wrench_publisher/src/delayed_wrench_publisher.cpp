/**
 * @file delayed_wrench_publisher.cpp
 * @brief Implementation of the DelayedWrenchPublisher node.
 */

#include "delayed_wrench_publisher/delayed_wrench_publisher.hpp"

using namespace std::chrono_literals;

DelayedWrenchPublisher::DelayedWrenchPublisher()
: Node("delayed_wrench_publisher"), is_triggered_(false)
{
  // Initialize message data
  init_wrench_plan_data();

  // Create Publisher
  publisher_ =
    this->create_publisher<planner_msgs::msg::WrenchPlan>("planner/wrench_planner/wrench_plan", 10);

  // Create Subscriber
  subscription_ = this->create_subscription<planner_msgs::msg::PDLAFeedback>(
    "/planner/pdla_planner/pdla_feedback", 10,
    std::bind(&DelayedWrenchPublisher::feedback_callback, this, std::placeholders::_1));

  // 1. Initial Delay Timer (10s)
  delay_timer_ =
    this->create_wall_timer(10s, std::bind(&DelayedWrenchPublisher::delay_timer_callback, this));
  delay_timer_->cancel();

  // 2. Stop Timer (30s duration)
  stop_timer_ =
    this->create_wall_timer(30s, std::bind(&DelayedWrenchPublisher::stop_timer_callback, this));
  stop_timer_->cancel();

  // 3. Publication Timer (5Hz)
  publish_timer_ = this->create_wall_timer(
    200ms, std::bind(&DelayedWrenchPublisher::publish_timer_callback, this));
  publish_timer_->cancel();

  RCLCPP_INFO(this->get_logger(), "Node initialized. Waiting for first feedback...");
}

void DelayedWrenchPublisher::init_wrench_plan_data()
{
  // Set only the requested fields.
  // Other fields are initialized to 0, false, or empty string by the message constructor.

  // --- Priority ---
  wrench_msg_.priority.id = 255;
  wrench_msg_.priority.timeout_ms = 5000;

  // --- Flags ---
  wrench_msg_.has_master = true;
  wrench_msg_.has_slave = false;

  // --- Targets ---
  wrench_msg_.targets.x = 1.0;
  wrench_msg_.targets.z = 2.5;
}

void DelayedWrenchPublisher::feedback_callback(const planner_msgs::msg::PDLAFeedback::SharedPtr msg)
{
  (void)msg;  // Unused

  // Only trigger on the first feedback received
  if (!is_triggered_) {
    RCLCPP_INFO(this->get_logger(), "First feedback received. Waiting 10s...");
    is_triggered_ = true;
    delay_timer_->reset();  // Start the 10s countdown
  }
}

void DelayedWrenchPublisher::delay_timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "10s passed. Starting 5Hz publication for 30s.");

  // Stop the initial delay timer
  delay_timer_->cancel();

  // Start the periodic publication
  publish_timer_->reset();

  // Start the stop timer (counts 30s from now)
  stop_timer_->reset();
}

void DelayedWrenchPublisher::stop_timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "30s duration passed. Stopping publication.");

  // Stop the publication
  publish_timer_->cancel();

  // Stop this timer (one-shot)
  stop_timer_->cancel();
}

void DelayedWrenchPublisher::publish_timer_callback()
{
  // Update timestamp
  wrench_msg_.header.stamp = this->now();

  // Publish
  publisher_->publish(wrench_msg_);
}

/**
 * @brief Main entry point.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DelayedWrenchPublisher>());
  rclcpp::shutdown();
  return 0;
}
