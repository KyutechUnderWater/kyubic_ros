/**
 * @file pdla_feedback_repub.cpp
 * @brief Header for PDLA Action Feedback Republisher.
 * Subscribes to action feedback topic and republishes as a simple message.
 * @author R.Ohnishi
 * @date 2026/01/29
 *
 * @details jazzyはaction feedback topicのbagを取れないので再パブリッシュ
 *********************************************************************************/

#include "projection_dynamic_look_ahead_planner/pdla_feedback_repub.hpp"

namespace planner::pdla_planner
{
PDLAFeedbackRepub::PDLAFeedbackRepub(const rclcpp::NodeOptions & options)
: Node("pdla_feedback_repub", options)
{
  action_feedback_sub_ = this->create_subscription<ActionFeedbackMsg>(
    "feedback", 10, std::bind(&PDLAFeedbackRepub::feedback_callback, this, std::placeholders::_1));

  custom_feedback_pub_ = this->create_publisher<CustomFeedbackMsg>("pdla_feedback", 10);
}

PDLAFeedbackRepub::~PDLAFeedbackRepub() {}

void PDLAFeedbackRepub::feedback_callback(const ActionFeedbackMsg::SharedPtr msg)
{
  CustomFeedbackMsg output_msg;

  // Copy data from the action feedback to the custom message.
  output_msg.header = msg->feedback.header;
  output_msg.csv_file_path = msg->feedback.csv_file_path;
  output_msg.step_idx = msg->feedback.step_idx;
  output_msg.step_state = msg->feedback.step_state;
  output_msg.odom = msg->feedback.odom;

  custom_feedback_pub_->publish(output_msg);

  RCLCPP_DEBUG(this->get_logger(), "Republished feedback for step: %d", output_msg.step_idx);
}

}  // namespace planner::pdla_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planner::pdla_planner::PDLAFeedbackRepub)
