#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <cmath>
#include <vector>
#include <utility>

using std::placeholders::_1;
using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class WaypointFollower : public rclcpp::Node
{
public:
    WaypointFollower()
    : Node("waypoint_follower"), current_index_(0), is_waiting_for_result_(false)
    {
        // 왕복용 waypoints 설정 (5회 왕복 = 10개 지점)
        for (int i = 0; i < 5; ++i) {
            waypoints_.emplace_back(1.1, 0.5);
            waypoints_.emplace_back(1.9, 0.2);
        }

        arrival_threshold_ = 0.04;

        // action client
        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // pose subscriber
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tracked_pose", 10, std::bind(&WaypointFollower::pose_callback, this, _1));

        timer_ = this->create_wall_timer(
            1s, std::bind(&WaypointFollower::send_next_goal, this));

        RCLCPP_INFO(this->get_logger(), "Waiting for action server to be available...");
    }

private:
    void send_next_goal()
    {
        if (!action_client_->wait_for_action_server(0s)) {
            RCLCPP_WARN(this->get_logger(), "Action server not available");
            return;
        }

        if (is_waiting_for_result_ || current_index_ >= waypoints_.size()) return;

        double x = waypoints_[current_index_].first;
        double y = waypoints_[current_index_].second;

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Sending goal: (%.2f, %.2f)", x, y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Goal failed or canceled.");
                }
            };

        action_client_->async_send_goal(goal_msg, send_goal_options);
        is_waiting_for_result_ = true;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (current_index_ >= waypoints_.size()) return;

        double current_x = msg->pose.position.x;
        double current_y = msg->pose.position.y;

        double target_x = waypoints_[current_index_].first;
        double target_y = waypoints_[current_index_].second;

        double dist = std::hypot(target_x - current_x, target_y - current_y);
        if (dist < arrival_threshold_) {
            RCLCPP_INFO(this->get_logger(), "Arrived at (%.2f, %.2f)", target_x, target_y);
            current_index_++;
            is_waiting_for_result_ = false;

            if (current_index_ < waypoints_.size()) {
                RCLCPP_INFO(this->get_logger(), "Next target: (%.2f, %.2f)",
                            waypoints_[current_index_].first, waypoints_[current_index_].second);
            } else {
                RCLCPP_INFO(this->get_logger(), "All waypoints completed.");
            }
        }
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::pair<double, double>> waypoints_;
    size_t current_index_;
    double arrival_threshold_;
    bool is_waiting_for_result_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointFollower>());
    rclcpp::shutdown();
    return 0;
}
