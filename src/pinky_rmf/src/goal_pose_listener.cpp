#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class GoalPoseListener : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  GoalPoseListener()
  : Node("goal_pose_listener")
  {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose",
      10,
      std::bind(&GoalPoseListener::goal_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waiting for goal_pose topic...");
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose = *msg;

    RCLCPP_INFO(this->get_logger(), "Sending goal to Nav2: x=%.2f, y=%.2f",
                goal.pose.pose.position.x, goal.pose.pose.position.y);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback =
      std::bind(&GoalPoseListener::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&GoalPoseListener::result_callback, this, std::placeholders::_1);

    action_client_->async_send_goal(goal, send_goal_options);
  }

  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    const auto & pos = feedback->current_pose.pose.position;
    RCLCPP_INFO(this->get_logger(), "Feedback: x=%.2f, y=%.2f", pos.x, pos.y);
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Navigation aborted.");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Navigation canceled.");
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "Unknown result code");
        break;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalPoseListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
