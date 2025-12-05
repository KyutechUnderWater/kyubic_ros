#include "wrench_action_sample/wrench_action_server.hpp"

#include <rclcpp/rclcpp.hpp>

WrenchActionServer::WrenchActionServer(const rclcpp::NodeOptions & options)
: Node("wrench_action_server", options)
{
  using namespace std::placeholders;

  this->action_server_ = rclcpp_action::create_server<Wrench>(
    this, "wrench_action", std::bind(&WrenchActionServer::handle_goal, this, _1, _2),
    std::bind(&WrenchActionServer::handle_cancel, this, _1),
    std::bind(&WrenchActionServer::handle_accepted, this, _1));

  this->publisher_ = this->create_publisher<WrenchMsg>("robot_force", 10);

  RCLCPP_INFO(
    this->get_logger(),
    "Wrench Action Server started. Waiting for requests on '/wrench_action'...");
}

rclcpp_action::GoalResponse WrenchActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Wrench::Goal> goal)
{
  RCLCPP_INFO(
    this->get_logger(), "Received request to publish Wrench for %d seconds", goal->duration_sec);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WrenchActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleApplyWrench> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WrenchActionServer::handle_accepted(const std::shared_ptr<GoalHandleApplyWrench> goal_handle)
{
  // Execute in a separate thread to avoid blocking the executor
  std::thread{std::bind(&WrenchActionServer::execute, this, std::placeholders::_1), goal_handle}
    .detach();
}

void WrenchActionServer::execute(const std::shared_ptr<GoalHandleApplyWrench> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Starting task execution...");

  rclcpp::Rate loop_rate(10.0);  // 10Hz
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Wrench::Feedback>();
  auto result = std::make_shared<Wrench::Result>();
  auto msg_to_publish = std::make_unique<WrenchMsg>();

  auto start_time = this->get_clock()->now();
  int count = 0;

  while (rclcpp::ok() && (this->get_clock()->now() - start_time) <
                           rclcpp::Duration::from_seconds(goal->duration_sec)) {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->total_updates = count;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Task canceled");
      return;
    }

    // Populate the message
    msg_to_publish->header.stamp = this->get_clock()->now();
    msg_to_publish->header.frame_id = "base_link";
    msg_to_publish->wrench.force.x = 1.0 * count;
    msg_to_publish->wrench.force.y = 2.0 * count;
    msg_to_publish->wrench.force.z = 0.0;
    msg_to_publish->wrench.torque.x = 0.0;
    msg_to_publish->wrench.torque.y = 0.0;
    msg_to_publish->wrench.torque.z = 0.0;

    // Update feedback
    feedback->current_wrench = *msg_to_publish;

    // Publish message and feedback
    publisher_->publish(std::move(msg_to_publish));
    goal_handle->publish_feedback(feedback);

    count++;
    msg_to_publish = std::make_unique<WrenchMsg>();  // Prepare new message container
    loop_rate.sleep();
  }

  // Check if the node is still running before declaring success
  if (rclcpp::ok()) {
    result->total_updates = count;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Task completed (published %d times)", count);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<WrenchActionServer>();

  // Use MultiThreadedExecutor to handle callbacks (like cancel) while execute() is running
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
