// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "mapf_planner/cbs.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"

// #include <vector>
// #include <map>
// #include <utility>
// #include <cmath>

// using NavigateToPose = nav2_msgs::action::NavigateToPose;
// using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
// using namespace std::chrono_literals;

// class CBSPlannerNode : public rclcpp::Node {
// public:
//     CBSPlannerNode() : Node("cbs_planner_node_test") {
//         RCLCPP_INFO(this->get_logger(), "CBS Planner Node Started");

//         // ========== Map ==========
//         map_ = {
//             {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//             {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
//         };

//         resolution_ = 0.1;
//         arrival_threshold_ = 0.02;

//         starts_ = {{4, 19}, {3, 19}, {2, 19}};
//         goals_  = {{5, 13}, {5, 12}, {5, 11}};

//         CBSSolver solver(map_, starts_, goals_);
//         paths_ = solver.findSolution(true);
//         if (paths_.empty()) {
//             RCLCPP_ERROR(this->get_logger(), "CBS failed to find paths.");
//             return;
//         }

//         for (size_t i = 0; i < paths_.size(); ++i) {
//             int id = i + 1;

//             // /aruco_pose{id} subscriber
//             auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//                 "/aruco_pose" + std::to_string(id), 10,
//                 [this, id](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//                     current_poses_[id] = *msg;
//                 });
//             pose_subs_.push_back(sub);

//             // Action client for each robot
//             auto client = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
//             action_clients_[id] = client;

//             // Marker publisher
//             auto marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
//                 "/waypoints_marker" + std::to_string(id), 10);
//             marker_publishers_[id] = marker_pub;

//             std::vector<std::pair<double, double>> wp;
//             for (const auto& p : paths_[i]) {
//                 double x = (p.second - 1) * resolution_ + resolution_ / 2.0;
//                 double y = (p.first - 1) * resolution_ + resolution_ / 2.0;
//                 wp.emplace_back(x, y);
//             }
//             waypoints_[id] = wp;
//             waypoint_index_[id] = 0;
//             is_waiting_[id] = false;
//         }

//         timer_ = this->create_wall_timer(500ms, std::bind(&CBSPlannerNode::process_waypoints, this));
//     }

// private:
//     // --- yaw 계산 함수 ---
//     double calcYaw(const std::pair<double, double>& from, const std::pair<double, double>& to) {
//         double dx = to.first  - from.first;
//         double dy = to.second - from.second;
//         return std::atan2(dy, dx);
//     }

//     // --- yaw를 쿼터니언에 반영하는 함수 ---
//     void setYawToQuaternion(geometry_msgs::msg::Quaternion& q, double yaw) {
//         q.x = 0.0;
//         q.y = 0.0;
//         q.z = std::sin(yaw / 2.0);
//         q.w = std::cos(yaw / 2.0);
//     }

//     void process_waypoints() {
//         for (const auto& [id, wp_list] : waypoints_) {
//             // 1) 아직 남은 waypoint가 있는지 체크
//             if (waypoint_index_[id] >= wp_list.size()) {
//                 RCLCPP_INFO(this->get_logger(),
//                             "[Robot %d] 모든 waypoint를 완료함 (index=%zu, 전체=%zu)",
//                             id, waypoint_index_[id], wp_list.size());
//                 continue;
//             } else {
//                 RCLCPP_INFO(this->get_logger(),
//                             "[Robot %d] 남은 waypoint가 있음 (index=%zu, 전체=%zu)",
//                             id, waypoint_index_[id], wp_list.size());
//             }

//             // 2) 도착 대기중인지(is_waiting) 체크
//             if (is_waiting_[id]) {
//                 RCLCPP_INFO(this->get_logger(),
//                             "[Robot %d] 현재 Nav2 액션 결과 대기중 (is_waiting=true)", id);
//                 continue;
//             } else {
//                 RCLCPP_INFO(this->get_logger(),
//                             "[Robot %d] 도착 대기중 아님 (is_waiting=false)", id);
//             }

//             // 3) 현재 pose를 받았는지 체크
//             if (current_poses_.count(id) == 0) {
//                 RCLCPP_INFO(this->get_logger(),
//                             "[Robot %d] 아직 /aruco_pose%d 메시지를 받은 적 없음", id, id);
//                 continue;
//             } else {
//                 RCLCPP_INFO(this->get_logger(),
//                             "[Robot %d] /aruco_pose%d 메시지를 받음", id, id);
//             }

//             auto& current = current_poses_[id];
//             auto [target_x, target_y] = wp_list[waypoint_index_[id]];
//             double dx = current.pose.position.x - target_x;
//             double dy = current.pose.position.y - target_y;

//             // (도달 판단도 로그로 추가)
//             double dist = std::hypot(dx, dy);
//             RCLCPP_INFO(this->get_logger(),
//                         "[Robot %d] 현재위치(%.2f, %.2f), 목표(%.2f, %.2f), 거리=%.3f",
//                         id, current.pose.position.x, current.pose.position.y, target_x, target_y, dist);

//             if (dist < arrival_threshold_) {
//                 RCLCPP_INFO(this->get_logger(),
//                             "[Robot %d] waypoint(%.2f, %.2f)에 도달 (threshold=%.2f)",
//                             id, target_x, target_y, arrival_threshold_);
//                 waypoint_index_[id]++;
//                 if (waypoint_index_[id] < wp_list.size()) {
//                     auto [next_x, next_y] = wp_list[waypoint_index_[id]];
//                     RCLCPP_INFO(this->get_logger(),
//                         "[Robot %d] >>> 다음 waypoint로 이동: (%.2f, %.2f) [index=%zu/%zu]",
//                         id, next_x, next_y, waypoint_index_[id], wp_list.size());
//                 } else {
//                     RCLCPP_INFO(this->get_logger(),
//                         "[Robot %d] >>> 모든 waypoint 완료!", id);
//                 }
//                 is_waiting_[id] = false;
//             }

//             if (waypoint_index_[id] < wp_list.size() && !is_waiting_[id]) {
//                 RCLCPP_INFO(this->get_logger(),
//                             "[Robot %d] Nav2에 goal 전달: (%.2f, %.2f)",
//                             id, wp_list[waypoint_index_[id]].first, wp_list[waypoint_index_[id]].second);
//                 send_goal(id, wp_list[waypoint_index_[id]]);
//                 is_waiting_[id] = true;
//             }

//             publish_markers(id);
//         }
//     }

//     void send_goal(int id, const std::pair<double, double>& target) {
//         auto client = action_clients_[id];
//         if (!client->wait_for_action_server(1s)) {
//             RCLCPP_WARN(this->get_logger(), "Action server not available for robot %d", id);
//             return;
//         }

//         NavigateToPose::Goal goal_msg;
//         goal_msg.pose.header.frame_id = "map";
//         goal_msg.pose.header.stamp = this->now();
//         goal_msg.pose.pose.position.x = target.first;
//         goal_msg.pose.pose.position.y = target.second;

//         // === 방향(yaw) 계산 및 적용 ===
//         size_t idx = waypoint_index_[id];
//         double yaw = 0.0;
//         if (idx > 0 && idx < waypoints_[id].size()) {
//             // 이전 waypoint에서 현재 waypoint를 향하는 방향
//             const auto& prev = waypoints_[id][idx - 1];
//             yaw = calcYaw(prev, target);
//         } else if (idx + 1 < waypoints_[id].size()) {
//             // 첫번째 waypoint라면 다음 waypoint 방향으로 설정
//             const auto& next = waypoints_[id][idx + 1];
//             yaw = calcYaw(target, next);
//         }
//         setYawToQuaternion(goal_msg.pose.pose.orientation, yaw);

//         auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
//         client->async_send_goal(goal_msg, options);

//         RCLCPP_INFO(this->get_logger(), "Sent goal to robot %d: (%.2f, %.2f), yaw=%.2f°",
//                     id, target.first, target.second, yaw * 180.0 / M_PI);
//     }

//     void publish_markers(int id) {
//         visualization_msgs::msg::MarkerArray marker_array;
//         int i = 0;
//         for (size_t idx = waypoint_index_[id]; idx < waypoints_[id].size(); ++idx) {
//             const auto& wp = waypoints_[id][idx];
//             visualization_msgs::msg::Marker marker;
//             marker.header.frame_id = "map";
//             marker.header.stamp = this->now();
//             marker.ns = "waypoints_" + std::to_string(id);
//             marker.id = i++;
//             marker.type = visualization_msgs::msg::Marker::SPHERE;
//             marker.action = visualization_msgs::msg::Marker::ADD;
//             marker.pose.position.x = wp.first;
//             marker.pose.position.y = wp.second;
//             marker.pose.position.z = 0.05;
//             marker.scale.x = 0.05;
//             marker.scale.y = 0.05;
//             marker.scale.z = 0.05;
//             marker.color.r = 1.0f;
//             marker.color.g = 0.5f;
//             marker.color.b = 0.0f;
//             marker.color.a = 1.0f;
//             marker.lifetime = rclcpp::Duration::from_seconds(1.0);
//             marker_array.markers.push_back(marker);
//         }
//         marker_publishers_[id]->publish(marker_array);
//     }

//     std::vector<std::vector<bool>> map_;
//     std::vector<Position> starts_, goals_;
//     std::vector<std::vector<Position>> paths_;
//     double resolution_, arrival_threshold_;

//     std::map<int, std::vector<std::pair<double, double>>> waypoints_;
//     std::map<int, size_t> waypoint_index_;
//     std::map<int, geometry_msgs::msg::PoseStamped> current_poses_;
//     std::map<int, bool> is_waiting_;
//     std::map<int, rclcpp_action::Client<NavigateToPose>::SharedPtr> action_clients_;
//     std::map<int, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> marker_publishers_;
//     std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<CBSPlannerNode>());
//     rclcpp::shutdown();
//     return 0;
// }

/////////////////////////////////////////////////////////////////////////////////////////////

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <vector>
#include <cmath>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class WaypointFollowerNode : public rclcpp::Node {
public:
    WaypointFollowerNode()
        : Node("waypoint_follower_node"), waypoint_idx_(0)
    {
        waypoints_ = {
            {1.85, 0.25}, {1.75, 0.25}, {1.65, 0.25},
            {1.55, 0.25}, {1.45, 0.25}, {1.35, 0.25}, {1.25, 0.25},
            {1.15, 0.25}, {1.15, 0.35}, {1.15, 0.45}
        };
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
        timer_ = this->create_wall_timer(3000ms, std::bind(&WaypointFollowerNode::process, this));
    }

private:
    void process() {
        if (waypoint_idx_ >= waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "All waypoints reached!");
            return;
        }
        send_goal_with_yaw(waypoint_idx_);
        waypoint_idx_++;
    }

    void send_goal_with_yaw(size_t idx) {
        if (!nav_client_->wait_for_action_server(1s)) {
            RCLCPP_WARN(this->get_logger(), "Nav2 action server not available!");
            return;
        }
        double x = waypoints_[idx].first;
        double y = waypoints_[idx].second;
        double yaw = 0.0;
        if (idx + 1 < waypoints_.size()) {
            double nx = waypoints_[idx + 1].first;
            double ny = waypoints_[idx + 1].second;
            yaw = std::atan2(ny - y, nx - x);
        } else if (idx > 0) {
            double px = waypoints_[idx - 1].first;
            double py = waypoints_[idx - 1].second;
            yaw = std::atan2(y - py, x - px);
        } else {
            yaw = 0.0;
        }
        NavigateToPose::Goal goal_msg;
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        setYawToQuaternion(goal_msg.pose.pose.orientation, yaw);

        nav_client_->async_send_goal(goal_msg);
        RCLCPP_INFO(this->get_logger(), "Goal sent: (%.2f, %.2f), yaw=%.2f°", x, y, yaw * 180.0 / M_PI);
    }

    void setYawToQuaternion(geometry_msgs::msg::Quaternion &q, double yaw) {
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw / 2.0);
        q.w = std::cos(yaw / 2.0);
    }

    std::vector<std::pair<double, double>> waypoints_;
    size_t waypoint_idx_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointFollowerNode>());
    rclcpp::shutdown();
    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "mapf_planner/cbs.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"

// #include <vector>
// #include <map>
// #include <utility>
// #include <cmath>

// using NavigateToPose = nav2_msgs::action::NavigateToPose;
// using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
// using namespace std::chrono_literals;

// struct WaypointWithYaw {
//     double x;
//     double y;
//     double yaw;
// };

// class CBSPlannerNode : public rclcpp::Node {
// public:
//     CBSPlannerNode() : Node("cbs_planner_node_test") {
//         RCLCPP_INFO(this->get_logger(), "CBS Planner Node Started");

//         // ========== Map ==========
//         map_ = {
//             {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//             {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//             {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
//         };

//         resolution_ = 0.1;
//         arrival_threshold_ = 0.02;

//         starts_ = {{4, 19}, {3, 19}, {2, 19}};
//         goals_  = {{5, 13}, {5, 12}, {5, 11}};

//         CBSSolver solver(map_, starts_, goals_);
//         paths_ = solver.findSolution(true);
//         if (paths_.empty()) {
//             RCLCPP_ERROR(this->get_logger(), "CBS failed to find paths.");
//             return;
//         }

//         constexpr double GOAL_FIXED_YAW = M_PI / 2.0; // 북쪽

//         for (size_t i = 0; i < paths_.size(); ++i) {
//             int id = i + 1;

//             // /aruco_pose{id} subscriber
//             auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//                 "/aruco_pose" + std::to_string(id), 10,
//                 [this, id](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//                     current_poses_[id] = *msg;
//                 });
//             pose_subs_.push_back(sub);

//             // Action client for each robot
//             auto client = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
//             action_clients_[id] = client;

//             // Marker publisher
//             auto marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
//                 "/waypoints_marker" + std::to_string(id), 10);
//             marker_publishers_[id] = marker_pub;

//             // Waypoint 변환: (y, x) 인덱스 → (x, y, yaw) 리스트
//             std::vector<WaypointWithYaw> wp;
//             const auto& path = paths_[i];
//             for (size_t j = 0; j < path.size(); ++j) {
//                 double x = (path[j].second - 1) * resolution_ + resolution_ / 2.0;
//                 double y = (path[j].first  - 1) * resolution_ + resolution_ / 2.0;
//                 double yaw = 0.0;
//                 if (j + 1 < path.size()) {
//                     // 다음 waypoint 방향
//                     double nx = (path[j+1].second - 1) * resolution_ + resolution_ / 2.0;
//                     double ny = (path[j+1].first  - 1) * resolution_ + resolution_ / 2.0;
//                     yaw = std::atan2(ny - y, nx - x);
//                 } else if (j > 0) {
//                     // 마지막: 이전 방향(아래와 else절 중 한 개만 쓰세요)
//                     double px = (path[j-1].second - 1) * resolution_ + resolution_ / 2.0;
//                     double py = (path[j-1].first  - 1) * resolution_ + resolution_ / 2.0;
//                     // yaw = std::atan2(y - py, x - px); // 이전 방향
//                     yaw = GOAL_FIXED_YAW; // 고정 방향(북쪽)
//                 } else {
//                     // path size==1 (드물지만)
//                     yaw = GOAL_FIXED_YAW;
//                 }
//                 wp.push_back({x, y, yaw});
//             }
//             waypoints_[id] = wp;
//             waypoint_index_[id] = 0;
//             is_waiting_[id] = false;
//         }

//         timer_ = this->create_wall_timer(500ms, std::bind(&CBSPlannerNode::process_waypoints, this));
//     }

// private:
//     void setYawToQuaternion(geometry_msgs::msg::Quaternion& q, double yaw) {
//         q.x = 0.0;
//         q.y = 0.0;
//         q.z = std::sin(yaw / 2.0);
//         q.w = std::cos(yaw / 2.0);
//     }

//     void process_waypoints() {
//         for (const auto& [id, wp_list] : waypoints_) {
//             // if (waypoint_index_[id] >= wp_list.size()) {
//             //     RCLCPP_INFO(this->get_logger(),
//             //                 "[Robot %d] 모든 waypoint를 완료함 (index=%zu, 전체=%zu)",
//             //                 id, waypoint_index_[id], wp_list.size());
//             //     continue;
//             // }
//             // if (is_waiting_[id]) {
//             //     RCLCPP_INFO(this->get_logger(),
//             //                 "[Robot %d] 현재 Nav2 액션 결과 대기중 (is_waiting=true)", id);
//             //     continue;
//             // }
//             // if (current_poses_.count(id) == 0) {
//             //     RCLCPP_INFO(this->get_logger(),
//             //                 "[Robot %d] 아직 /aruco_pose%d 메시지를 받은 적 없음", id, id);
//             //     continue;
//             // }

//             auto& current = current_poses_[id];
//             const auto& wp = wp_list[waypoint_index_[id]];
//             double dx = current.pose.position.x - wp.x;
//             double dy = current.pose.position.y - wp.y;
//             double dist = std::hypot(dx, dy);

//             RCLCPP_INFO(this->get_logger(),
//                         "[Robot %d] 현재위치(%.2f, %.2f), 목표(%.2f, %.2f), 거리=%.3f",
//                         id, current.pose.position.x, current.pose.position.y, wp.x, wp.y, dist);

//             if (dist < arrival_threshold_) {
//                 RCLCPP_INFO(this->get_logger(),
//                             "[Robot %d] waypoint(%.2f, %.2f)에 도달 (threshold=%.2f)",
//                             id, wp.x, wp.y, arrival_threshold_);
//                 waypoint_index_[id]++;
//                 if (waypoint_index_[id] < wp_list.size()) {
//                     const auto& next_wp = wp_list[waypoint_index_[id]];
//                     RCLCPP_INFO(this->get_logger(),
//                         "[Robot %d] >>> 다음 waypoint로 이동: (%.2f, %.2f) [index=%zu/%zu]",
//                         id, next_wp.x, next_wp.y, waypoint_index_[id], wp_list.size());
//                 } else {
//                     RCLCPP_INFO(this->get_logger(),
//                         "[Robot %d] >>> 모든 waypoint 완료!", id);
//                 }
//                 is_waiting_[id] = false;
//             }

//             if (waypoint_index_[id] < wp_list.size() && !is_waiting_[id]) {
//                 RCLCPP_INFO(this->get_logger(),
//                             "[Robot %d] Nav2에 goal 전달: (%.2f, %.2f, yaw=%.2f°)",
//                             id, wp.x, wp.y, wp.yaw * 180.0 / M_PI);
//                 send_goal(id, wp);
//                 is_waiting_[id] = true;
//             }

//             publish_markers(id);
//         }
//     }

//     void send_goal(int id, const WaypointWithYaw& target) {
//         auto client = action_clients_[id];
//         if (!client->wait_for_action_server(1s)) {
//             RCLCPP_WARN(this->get_logger(), "Action server not available for robot %d", id);
//             return;
//         }

//         NavigateToPose::Goal goal_msg;
//         goal_msg.pose.header.frame_id = "map";
//         goal_msg.pose.header.stamp = this->now();
//         goal_msg.pose.pose.position.x = target.x;
//         goal_msg.pose.pose.position.y = target.y;
//         setYawToQuaternion(goal_msg.pose.pose.orientation, target.yaw);

//         auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
//         client->async_send_goal(goal_msg, options);

//         RCLCPP_INFO(this->get_logger(), "Sent goal to robot %d: (%.2f, %.2f), yaw=%.2f°",
//                     id, target.x, target.y, target.yaw * 180.0 / M_PI);
//     }

//     void publish_markers(int id) {
//         visualization_msgs::msg::MarkerArray marker_array;
//         int i = 0;
//         for (size_t idx = waypoint_index_[id]; idx < waypoints_[id].size(); ++idx) {
//             const auto& wp = waypoints_[id][idx];
//             visualization_msgs::msg::Marker marker;
//             marker.header.frame_id = "map";
//             marker.header.stamp = this->now();
//             marker.ns = "waypoints_" + std::to_string(id);
//             marker.id = i++;
//             marker.type = visualization_msgs::msg::Marker::SPHERE;
//             marker.action = visualization_msgs::msg::Marker::ADD;
//             marker.pose.position.x = wp.x;
//             marker.pose.position.y = wp.y;
//             marker.pose.position.z = 0.05;
//             marker.scale.x = 0.05;
//             marker.scale.y = 0.05;
//             marker.scale.z = 0.05;
//             marker.color.r = 1.0f;
//             marker.color.g = 0.5f;
//             marker.color.b = 0.0f;
//             marker.color.a = 1.0f;
//             marker.lifetime = rclcpp::Duration::from_seconds(1.0);
//             marker_array.markers.push_back(marker);
//         }
//         marker_publishers_[id]->publish(marker_array);
//     }

//     std::vector<std::vector<bool>> map_;
//     std::vector<Position> starts_, goals_;
//     std::vector<std::vector<Position>> paths_;
//     double resolution_, arrival_threshold_;

//     std::map<int, std::vector<WaypointWithYaw>> waypoints_; // <--- 변경
//     std::map<int, size_t> waypoint_index_;
//     std::map<int, geometry_msgs::msg::PoseStamped> current_poses_;
//     std::map<int, bool> is_waiting_;
//     std::map<int, rclcpp_action::Client<NavigateToPose>::SharedPtr> action_clients_;
//     std::map<int, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> marker_publishers_;
//     std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<CBSPlannerNode>());
//     rclcpp::shutdown();
//     return 0;
// }
///////////////////////////////////////////////////////////////////////////////////////////
// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "mapf_planner/cbs.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"

// #include <vector>
// #include <map>
// #include <utility>
// #include <cmath>

// using NavigateToPose = nav2_msgs::action::NavigateToPose;
// using namespace std::chrono_literals;  // <<< 여기에 추가

// struct WaypointWithYaw {
//   double x, y, yaw;
// };

// struct RobotData {
//   int id;
//   std::vector<WaypointWithYaw> waypoints;
//   size_t waypoint_index{0};
//   geometry_msgs::msg::PoseStamped current_pose;
//   rclcpp_action::Client<NavigateToPose>::SharedPtr action_client;
//   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
//   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
// };

// class CBSPlannerNode : public rclcpp::Node {
// public:
//   CBSPlannerNode() : Node("cbs_planner_node_refactored") {
//     RCLCPP_INFO(get_logger(), "CBS Planner Node Refactored Start");

//     // --- 1. 맵 & 경로 계산 ---
//     std::vector<std::vector<bool>> map = {
//       {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//       {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
//     };
//     std::vector<Position> starts = {{4,19},{3,19},{2,19}};
//     std::vector<Position> goals  = {{5,13},{5,12},{5,11}};
//     CBSSolver solver(map, starts, goals);
//     auto paths = solver.findSolution(true);
//     if (paths.empty()) {
//       RCLCPP_ERROR(get_logger(), "CBS failed.");
//       return;
//     }

//     constexpr double RES = 0.1, GOAL_YAW = M_PI/2.0;
//     for (size_t i = 0; i < paths.size(); ++i) {
//       int id = i + 1;
//       RobotData rd;
//       rd.id = id;
//       rd.action_client = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
//       rd.marker_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
//         "/waypoints_marker" + std::to_string(id), 10);

//       // <<< QoS와 콜백 시그니처 수정
//       rd.pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
//         "/aruco_pose" + std::to_string(id),
//         rclcpp::QoS(10),
//         [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            
//           robots_[id].current_pose = *msg;
//         });

//       // 웨이포인트 생성
//       for (size_t j = 0; j < paths[i].size(); ++j) {
//         double x = (paths[i][j].second - 1) * RES + RES/2.0;
//         double y = (paths[i][j].first  - 1) * RES + RES/2.0;
//         double yaw = (j + 1 < paths[i].size())
//           ? std::atan2(
//               (paths[i][j+1].first - 1) * RES + RES/2.0 - y,
//               (paths[i][j+1].second - 1) * RES + RES/2.0 - x)
//           : GOAL_YAW;
//         rd.waypoints.push_back({x, y, yaw});
//       }
//       robots_[id] = rd;
//     }

//     // 2. 첫 웨이포인트 전송
//     for (auto & [id, rd] : robots_) {
//       send_next_goal(id);
//     }

//     // 3. 마커 퍼블리싱 타이머 (200ms)
//     marker_timer_ = create_wall_timer(200ms, [this]() {
//       for (auto & [id, rd] : robots_) {
//         publish_markers(rd);
//       }
//     });
//   }

// private:
//   void set_quat(geometry_msgs::msg::Quaternion &q, double yaw) {
//     q.x = q.y = 0.0;
//     q.z = std::sin(yaw / 2.0);
//     q.w = std::cos(yaw / 2.0);
//   }

//   void send_next_goal(int id) {
//     auto & rd = robots_[id];
//     if (rd.waypoint_index >= rd.waypoints.size()) {
//       RCLCPP_INFO(get_logger(), "[R%d] all waypoints done", id);
//       return;
//     }
//     // <<< 1s 사용 가능
//     if (!rd.action_client->wait_for_action_server(1s)) {
//       RCLCPP_WARN(get_logger(), "[R%d] action server not available", id);
//       return;
//     }

//     const auto & wp = rd.waypoints[rd.waypoint_index];
//     NavigateToPose::Goal goal;
//     goal.pose.header.frame_id = "map";
//     goal.pose.header.stamp = now();
//     goal.pose.pose.position.x = wp.x;
//     goal.pose.pose.position.y = wp.y;
//     set_quat(goal.pose.pose.orientation, wp.yaw);

//     auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
//     options.goal_response_callback =
//       [this, id](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr handle) {
//         if (!handle) {
//           RCLCPP_ERROR(get_logger(), "[R%d] goal rejected", id);
//         }
//       };
//     options.result_callback =
//       [this, id](const typename rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & res) {
//         (void)res; 
//         RCLCPP_INFO(get_logger(), "[R%d] reached waypoint %zu", id, robots_[id].waypoint_index);
//         robots_[id].waypoint_index++;
//         send_next_goal(id);
//       };

//     rd.action_client->async_send_goal(goal, options);
//     RCLCPP_INFO(get_logger(),
//                 "[R%d] sent waypoint %zu → (%.2f, %.2f)",
//                 id, rd.waypoint_index, wp.x, wp.y);
//   }

//   void publish_markers(const RobotData & rd) {
//     visualization_msgs::msg::MarkerArray array;
//     int cnt = 0;
//     for (size_t idx = rd.waypoint_index; idx < rd.waypoints.size(); ++idx) {
//       const auto & wp = rd.waypoints[idx];
//       visualization_msgs::msg::Marker m;
//       m.header.frame_id = "map";
//       m.header.stamp = now();
//       m.ns = "wp" + std::to_string(rd.id);
//       m.id = cnt++;
//       m.type = visualization_msgs::msg::Marker::SPHERE;
//       m.pose.position.x = wp.x;
//       m.pose.position.y = wp.y;
//       m.pose.position.z = 0.05;
//       m.scale.x = m.scale.y = m.scale.z = 0.05;
//       m.color.r = 1.0f; m.color.g = 0.5f; m.color.b = 0.0f; m.color.a = 1.0f;
//       array.markers.push_back(m);
//     }
//     robots_[rd.id].marker_pub->publish(array);
//   }

//   std::map<int, RobotData> robots_;
//   rclcpp::TimerBase::SharedPtr marker_timer_;
// };

// int main(int argc, char * argv[]) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<CBSPlannerNode>());
//   rclcpp::shutdown();
//   return 0;
// }
/////////////////////////////////////////////////////////////////////////
// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "mapf_planner/cbs.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"

// #include <vector>
// #include <cmath>
// #include <map>
// #include <string>

// using NavigateToPose = nav2_msgs::action::NavigateToPose;
// using namespace std::chrono_literals;

// struct WaypointWithYaw {
//   double x, y, yaw;
// };

// struct RobotData {
//   std::vector<WaypointWithYaw> waypoints;
//   size_t waypoint_index{0};
//   bool goal_sent{false};
//   geometry_msgs::msg::PoseStamped current_pose;
//   rclcpp_action::Client<NavigateToPose>::SharedPtr action_client;
//   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
//   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
//   rclcpp::TimerBase::SharedPtr timeout_timer;

//   static constexpr double THRESHOLD = 0.04;
//   static constexpr double THRESH_SQ = THRESHOLD * THRESHOLD;
// };

// class CBSPlannerNode : public rclcpp::Node {
// public:
//   CBSPlannerNode() : Node("cbs_planner_node_aruco3_timeout") {
//     RCLCPP_INFO(get_logger(), "CBS Planner Node (aruco_pose3 with timeout) Start");

//     std::vector<std::vector<bool>> map = {
//       {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//       {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
//       {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
//     };

//     std::vector<Position> starts = {{4,19},{3,19},{2,19}};
//     std::vector<Position> goals  = {{5,13},{5,12},{5,11}};
//     CBSSolver solver(map, starts, goals);
//     auto paths = solver.findSolution(true);
//     if (paths.size() < 3) {
//       RCLCPP_ERROR(get_logger(), "경로 계산 실패 또는 3번 로봇 경로 없음");
//       return;
//     }

//     constexpr double RES = 0.1;
//     robot3_.action_client = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose3");
//     robot3_.marker_pub = create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints_marker3", 10);
//     robot3_.pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
//       "/odom", rclcpp::QoS(10),
//       [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//         this->handle_pose(msg);
//       });

//     for (auto &cell : paths[2]) {
//       double x = (cell.second - 1) * RES + RES/2.0;
//       double y = (cell.first  - 1) * RES + RES/2.0;
//       double yaw = M_PI/2.0;
//       robot3_.waypoints.push_back({x,y,yaw});
//     }

//     marker_timer_ = create_wall_timer(200ms, [this]() { publish_markers(); });
//   }

// private:
//   void handle_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//     auto &rd = robot3_;
//     rd.current_pose = *msg;

//     if (!rd.goal_sent && rd.waypoint_index < rd.waypoints.size()) {
//       send_goal(rd.waypoints[rd.waypoint_index]);
//       rd.goal_sent = true;
//       start_timeout();
//       return;
//     }

//     const auto &wp = rd.waypoints[rd.waypoint_index];
//     double dx = rd.current_pose.pose.position.x - wp.x;
//     double dy = rd.current_pose.pose.position.y - wp.y;
//     if (dx*dx + dy*dy < RobotData::THRESH_SQ) {
//       RCLCPP_INFO(get_logger(),
//                   "aruco_pose3 reached wp[%zu] (%.2f, %.2f), dist=%.3f",
//                   rd.waypoint_index, wp.x, wp.y, std::sqrt(dx*dx+dy*dy));
//       if (rd.timeout_timer) rd.timeout_timer->cancel();
//       advance_waypoint();
//     }
//   }

//   void send_goal(const WaypointWithYaw &wp) {
//     auto &rd = robot3_;
//     if (!rd.action_client->wait_for_action_server(1s)) {
//       RCLCPP_WARN(get_logger(),"[3] action server not available");
//       return;
//     }
//     NavigateToPose::Goal goal;
//     goal.pose.header.frame_id = "map";
//     goal.pose.header.stamp = now();
//     goal.pose.pose.position.x = wp.x;
//     goal.pose.pose.position.y = wp.y;
//     goal.pose.pose.orientation.z = std::sin(wp.yaw/2.0);
//     goal.pose.pose.orientation.w = std::cos(wp.yaw/2.0);

//     auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
//     opts.goal_response_callback =
//       [this](auto handle){
//         if (!handle) RCLCPP_ERROR(get_logger(),"[3] goal rejected");
//       };
//     rd.action_client->async_send_goal(goal, opts);

//     RCLCPP_INFO(get_logger(),
//                 "[3] Sent goal → (%.2f, %.2f)", wp.x, wp.y);
//   }

//   void start_timeout() {
//     auto &rd = robot3_;
//     if (rd.timeout_timer) {
//       rd.timeout_timer->cancel();
//     }
//     rd.timeout_timer = create_wall_timer(
//       3s,
//       [this]() {
//         auto &rd = robot3_;
//         RCLCPP_WARN(get_logger(), "[3] waypoint[%zu] timeout after 3s",
//                     rd.waypoint_index);
//         rd.timeout_timer->cancel();
//         advance_waypoint();
//       });
//   }

//   void advance_waypoint() {
//     auto &rd = robot3_;
//     rd.waypoint_index++;
//     rd.goal_sent = false;
//     if (rd.waypoint_index < rd.waypoints.size()) {
//       send_goal(rd.waypoints[rd.waypoint_index]);
//       rd.goal_sent = true;
//       start_timeout();
//     } else {
//       RCLCPP_INFO(get_logger(), "[3] all waypoints done");
//     }
//   }

//   void publish_markers() {
//     visualization_msgs::msg::MarkerArray arr;
//     int cnt = 0;
//     for (size_t i = robot3_.waypoint_index;
//          i < robot3_.waypoints.size(); ++i)
//     {
//       const auto &wp = robot3_.waypoints[i];
//       visualization_msgs::msg::Marker m;
//       m.header.frame_id = "map";
//       m.header.stamp = now();
//       m.ns = "wp_3";
//       m.id = cnt++;
//       m.type = visualization_msgs::msg::Marker::SPHERE;
//       m.pose.position.x = wp.x;
//       m.pose.position.y = wp.y;
//       m.pose.position.z = 0.05;
//       m.scale.x = m.scale.y = m.scale.z = 0.05;
//       m.color.r = 1.0f; m.color.g = 0.5f;
//       m.color.b = 0.0f; m.color.a = 1.0f;
//       arr.markers.push_back(m);
//     }
//     robot3_.marker_pub->publish(arr);
//   }

//   RobotData robot3_;
//   rclcpp::TimerBase::SharedPtr marker_timer_;
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<CBSPlannerNode>());
//   rclcpp::shutdown();
//   return 0;
// }