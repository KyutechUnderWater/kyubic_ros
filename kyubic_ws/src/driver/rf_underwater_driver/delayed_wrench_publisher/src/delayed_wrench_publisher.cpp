/**
 * @file delayed_wrench_publisher.cpp
 * @brief Implementation of the DelayedWrenchPublisher node.
 */

#include "delayed_wrench_publisher/delayed_wrench_publisher.hpp"

using namespace std::chrono_literals;

DelayedWrenchPublisher::DelayedWrenchPublisher()
: Node("delayed_wrench_publisher"), is_triggered_(false)
{
  // --- Declare and Get Parameters ---
  int priority = this->declare_parameter("priority", 255);
  int timeout_ms = this->declare_parameter("timeout_ms", 5000);
  initial_delay_sec = this->declare_parameter("initial_delay_sec", 10.0);
  execution_duration_sec = this->declare_parameter("execution_duration_sec", 30.0);
  publish_rate_hz = this->declare_parameter("publish_rate_hz", 5.0);
  double target_x = this->declare_parameter("target_x", 0.0);
  double target_y = this->declare_parameter("target_y", 0.0);

  // Convert times to std::chrono
  auto initial_delay = std::chrono::duration<double>(initial_delay_sec);
  auto execution_duration = std::chrono::duration<double>(execution_duration_sec);
  auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_hz);

  // --- Initialize message data ---
  // Pass priority and timeout_ms to the initialization function
  init_wrench_plan_data(priority, timeout_ms, target_x, target_y);

  // --- Create Publisher ---
  publisher_ = this->create_publisher<planner_msgs::msg::WrenchPlan>("wrench_plan", 10);

  // --- Create Subscriber ---
  subscription_ = this->create_subscription<planner_msgs::msg::PDLAFeedback>(
    "pdla_feedback", 10,
    std::bind(&DelayedWrenchPublisher::feedback_callback, this, std::placeholders::_1));

  // --- Initialize Timers (initially canceled) ---

  // 1. Initial Delay Timer (Use parameter)
  delay_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(initial_delay),
    std::bind(&DelayedWrenchPublisher::delay_timer_callback, this));
  delay_timer_->cancel();

  // 2. Stop Timer (Use parameter)
  stop_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(execution_duration),
    std::bind(&DelayedWrenchPublisher::stop_timer_callback, this));
  stop_timer_->cancel();

  // 3. Publication Timer (Use parameter)
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(publish_period),
    std::bind(&DelayedWrenchPublisher::publish_timer_callback, this));
  publish_timer_->cancel();

  RCLCPP_INFO(
    this->get_logger(), "Waiting for first feedback. Delay: %.1fs, Duration: %.1fs, Rate: %.1fHz",
    initial_delay_sec, execution_duration_sec, publish_rate_hz);
}

void DelayedWrenchPublisher::init_wrench_plan_data(
  int priority, int timeout_ms, double target_x, double target_y)
{
  // Set the requested fields using parameters.

  // --- Priority (From Param) ---
  wrench_msg_.priority.id = static_cast<uint8_t>(priority);
  wrench_msg_.priority.timeout_ms = static_cast<uint32_t>(timeout_ms);

  // --- Flags ---
  wrench_msg_.has_master = false;
  wrench_msg_.has_slave = false;

  // --- Targets ---
  wrench_msg_.targets.x = target_x;
  wrench_msg_.targets.y = target_y;
}

void DelayedWrenchPublisher::feedback_callback(const planner_msgs::msg::PDLAFeedback::SharedPtr msg)
{
  (void)msg;  // Unused

  // Only trigger on the first feedback received
  if (!is_triggered_) {
    RCLCPP_INFO(this->get_logger(), "First feedback received. Waiting %.1fs...", initial_delay_sec);
    is_triggered_ = true;
    delay_timer_->reset();  // Start the 10s countdown
  }
}

void DelayedWrenchPublisher::delay_timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "  %.1fs passed.", initial_delay_sec);

  RCLCPP_INFO(
    this->get_logger(), "Starting %.1fHz publication for %.1fs. Target: %.2f, %.2f",
    publish_rate_hz, execution_duration_sec, wrench_msg_.targets.x, wrench_msg_.targets.y);

  // Stop the initial delay timer
  delay_timer_->cancel();

  // Start the periodic publication
  publish_timer_->reset();

  // Start the stop timer (counts 30s from now)
  stop_timer_->reset();
}

void DelayedWrenchPublisher::stop_timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "  %.1fs duration passed.", execution_duration_sec);

  RCLCPP_INFO(this->get_logger(), "Stopping publication.");

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
