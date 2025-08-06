#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mapf_planner/cbs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <vector>
#include <map>
#include <utility>
#include <cmath>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using namespace std::chrono_literals;

class CBSPlannerNode : public rclcpp::Node {
public:
    CBSPlannerNode() : Node("cbs_planner_node_test") {
        RCLCPP_INFO(this->get_logger(), "CBS Planner Node Started");

        // ========== Map ==========
        map_ = {
            {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
            {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
            {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
            {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
            {1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
            {1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
            {1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
            {1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
            {1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
            {1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
            {1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
            {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
        };

        resolution_ = 0.1;
        arrival_threshold_ = 0.05;

        starts_ = {{4, 19}, {3, 19}, {2, 19}};
        goals_  = {{5, 13}, {5, 12}, {5, 11}};

        CBSSolver solver(map_, starts_, goals_);
        paths_ = solver.findSolution(true);
        if (paths_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "CBS failed to find paths.");
            return;
        }

        for (size_t i = 0; i < paths_.size(); ++i) {
            int id = i + 1;

            // /aruco_pose{id} subscriber
            auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_pose" + std::to_string(id), 10,
                [this, id](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    current_poses_[id] = *msg;
                });
            pose_subs_.push_back(sub);

            // Action client for each robot
            auto client = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
            action_clients_[id] = client;

            // Marker publisher
            auto marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/waypoints_marker" + std::to_string(id), 10);
            marker_publishers_[id] = marker_pub;

            std::vector<std::pair<double, double>> wp;
            for (const auto& p : paths_[i]) {
                double x = (p.first - 1) * resolution_ + resolution_ / 2.0;
                double y = (p.second - 1) * resolution_ + resolution_ / 2.0;
                wp.emplace_back(x, y);
            }
            waypoints_[id] = wp;
            waypoint_index_[id] = 0;
            is_waiting_[id] = false;
        }

        timer_ = this->create_wall_timer(500ms, std::bind(&CBSPlannerNode::process_waypoints, this));
    }

private:
    void process_waypoints() {
        for (const auto& [id, wp_list] : waypoints_) {
            // 1) 아직 남은 waypoint가 있는지 체크
            if (waypoint_index_[id] >= wp_list.size()) {
                RCLCPP_INFO(this->get_logger(),
                            "[Robot %d] 모든 waypoint를 완료함 (index=%zu, 전체=%zu)",
                            id, waypoint_index_[id], wp_list.size());
                continue;
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "[Robot %d] 남은 waypoint가 있음 (index=%zu, 전체=%zu)",
                            id, waypoint_index_[id], wp_list.size());
            }

            // 2) 도착 대기중인지(is_waiting) 체크
            if (is_waiting_[id]) {
                RCLCPP_INFO(this->get_logger(),
                            "[Robot %d] 현재 Nav2 액션 결과 대기중 (is_waiting=true)", id);
                continue;
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "[Robot %d] 도착 대기중 아님 (is_waiting=false)", id);
            }

            // 3) 현재 pose를 받았는지 체크
            if (current_poses_.count(id) == 0) {
                RCLCPP_INFO(this->get_logger(),
                            "[Robot %d] 아직 /aruco_pose%d 메시지를 받은 적 없음", id, id);
                continue;
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "[Robot %d] /aruco_pose%d 메시지를 받음", id, id);
            }

            auto& current = current_poses_[id];
            auto [target_x, target_y] = wp_list[waypoint_index_[id]];
            double dx = current.pose.position.x - target_x;
            double dy = current.pose.position.y - target_y;

            // (도달 판단도 로그로 추가)
            double dist = std::hypot(dx, dy);
            RCLCPP_INFO(this->get_logger(),
                        "[Robot %d] 현재위치(%.2f, %.2f), 목표(%.2f, %.2f), 거리=%.3f",
                        id, current.pose.position.x, current.pose.position.y, target_x, target_y, dist);

            if (dist < arrival_threshold_) {
                RCLCPP_INFO(this->get_logger(),
                            "[Robot %d] waypoint(%.2f, %.2f)에 도달 (threshold=%.2f)",
                            id, target_x, target_y, arrival_threshold_);
                waypoint_index_[id]++;
                is_waiting_[id] = false;
            }

            if (waypoint_index_[id] < wp_list.size() && !is_waiting_[id]) {
                RCLCPP_INFO(this->get_logger(),
                            "[Robot %d] Nav2에 goal 전달: (%.2f, %.2f)",
                            id, wp_list[waypoint_index_[id]].first, wp_list[waypoint_index_[id]].second);
                send_goal(id, wp_list[waypoint_index_[id]]);
                is_waiting_[id] = true;
            }

            publish_markers(id);
        }
    }

    void send_goal(int id, const std::pair<double, double>& target) {
        auto client = action_clients_[id];
        if (!client->wait_for_action_server(1s)) {
            RCLCPP_WARN(this->get_logger(), "Action server not available for robot %d", id);
            return;
        }

        NavigateToPose::Goal goal_msg;
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = target.first;
        goal_msg.pose.pose.position.y = target.second;
        goal_msg.pose.pose.orientation.w = 1.0;

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        client->async_send_goal(goal_msg, options);

        RCLCPP_INFO(this->get_logger(), "Sent goal to robot %d: (%.2f, %.2f)", id, target.first, target.second);
    }

    void publish_markers(int id) {
        visualization_msgs::msg::MarkerArray marker_array;
        int i = 0;
        for (size_t idx = waypoint_index_[id]; idx < waypoints_[id].size(); ++idx) {
            const auto& wp = waypoints_[id][idx];
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "waypoints_" + std::to_string(id);
            marker.id = i++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = wp.first;
            marker.pose.position.y = wp.second;
            marker.pose.position.z = 0.05;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.r = 1.0f;
            marker.color.g = 0.5f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
            marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            marker_array.markers.push_back(marker);
        }
        marker_publishers_[id]->publish(marker_array);
    }

    std::vector<std::vector<bool>> map_;
    std::vector<Position> starts_, goals_;
    std::vector<std::vector<Position>> paths_;
    double resolution_, arrival_threshold_;

    std::map<int, std::vector<std::pair<double, double>>> waypoints_;
    std::map<int, size_t> waypoint_index_;
    std::map<int, geometry_msgs::msg::PoseStamped> current_poses_;
    std::map<int, bool> is_waiting_;
    std::map<int, rclcpp_action::Client<NavigateToPose>::SharedPtr> action_clients_;
    std::map<int, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> marker_publishers_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CBSPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
