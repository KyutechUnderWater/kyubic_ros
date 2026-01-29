/**
 * @file pdla_feedback_repub.hpp
 * @brief Header for PDLA Action Feedback Republisher.
 * Subscribes to action feedback topic and republishes as a simple message.
 * @author R.Ohnishi
 * @date 2026/01/29
 *
 * @details jazzyはaction feedback topicのbagを取れないので再パブリッシュ
 *********************************************************************************/

#ifndef _PDLA_FEEDBACK_REPUB_HPP
#define _PDLA_FEEDBACK_REPUB_HPP

#include "planner_msgs/action/pdla.hpp"
#include "planner_msgs/msg/pdla_feedback.hpp"
#include "rclcpp/rclcpp.hpp"

namespace planner::pdla_planner
{

/**
 * @class PDLAFeedbackRepub
 * @brief Node that converts Action Feedback topics to Custom Messages.
 */
class PDLAFeedbackRepub : public rclcpp::Node
{
public:
  // Action type definition
  using PDLA = planner_msgs::action::PDLA;
  using ActionFeedbackMsg = PDLA::Impl::FeedbackMessage;
  using CustomFeedbackMsg = planner_msgs::msg::PDLAFeedback;

  /**
   * @brief Construct a new PDLAFeedbackRepub object.
   * @param options Node options.
   */
  explicit PDLAFeedbackRepub(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~PDLAFeedbackRepub();

private:
  // Subscriber for the raw action feedback
  rclcpp::Subscription<ActionFeedbackMsg>::SharedPtr action_feedback_sub_;

  // Publisher for the custom message
  rclcpp::Publisher<CustomFeedbackMsg>::SharedPtr custom_feedback_pub_;

  /**
   * @brief Callback function for action feedback topic.
   * @param msg The raw action feedback message containing goal_id and feedback body.
   */
  void feedback_callback(const ActionFeedbackMsg::SharedPtr msg);
};
}  // namespace planner::pdla_planner

#endif  // PDLA_FEEDBACK_REPUB_HPP_
