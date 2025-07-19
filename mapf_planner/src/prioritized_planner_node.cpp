#include "rclcpp/rclcpp.hpp"
#include "mapf_planner/astar.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <memory>
#include <string>
#include <sstream>

using namespace std::chrono_literals;

class PrioritizedPlannerNode : public rclcpp::Node {
public:
    PrioritizedPlannerNode()
        : Node("prioritized_planner_node")
    {
        // 에이전트 start/goal 지정 (맵 크기에 맞게 수정 필요)
        agent_starts_ = { {1, 1}, {1, 10}, {20, 10} }; // 예시: 3개
        agent_goals_  = { {20, 10}, {20, 1}, {1, 1} };
        agent_num_ = agent_starts_.size();

        // 퍼블리셔를 생성자에서 한 번만 생성
        for (size_t i = 0; i < agent_num_; ++i) {
            std::string topic_name = "/waypoints" + std::to_string(i + 1);
            path_pubs_.push_back(this->create_publisher<nav_msgs::msg::Path>(topic_name, 10));
        }

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/grid_map", 10,
            std::bind(&PrioritizedPlannerNode::mapCallback, this, std::placeholders::_1)
        );
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received /grid_map: %d x %d", msg->info.width, msg->info.height);

        // 맵을 2D bool 배열로 변환
        std::vector<std::vector<bool>> map(msg->info.height, std::vector<bool>(msg->info.width));
        for (size_t y = 0; y < msg->info.height; ++y)
            for (size_t x = 0; x < msg->info.width; ++x)
                map[y][x] = (msg->data[y * msg->info.width + x] == 100);

        // 휴리스틱 함수
        auto get_heuristics = [&](const Position& goal) {
            std::vector<std::vector<int>> heuristics(msg->info.height, std::vector<int>(msg->info.width, 0));
            for (size_t y = 0; y < msg->info.height; ++y)
                for (size_t x = 0; x < msg->info.width; ++x)
                    heuristics[y][x] = std::abs(goal.first - static_cast<int>(y)) + std::abs(goal.second - static_cast<int>(x));
            return heuristics;
        };

        // Prioritized Planning with constraints 누적
        std::vector<Constraint> constraints;
        std::vector<std::vector<Position>> planned_paths(agent_num_);

        for (size_t i = 0; i < agent_num_; ++i) {
            auto heuristics = get_heuristics(agent_goals_[i]);
            AStarPlanner planner(map);
            auto path = planner.findPath(agent_starts_[i], agent_goals_[i], heuristics, i, constraints);

            if (path.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Agent %ld: No path found!", i+1);
                continue;
            }
            planned_paths[i] = path;

            // constraint 누적
            for (size_t t = 0; t < path.size(); ++t) {
                for (size_t j = 0; j < agent_num_; ++j) {
                    if (j == i) continue;
                    constraints.push_back(Constraint{static_cast<int>(j), static_cast<int>(t), {path[t]}});
                    if (t + 1 < path.size()) {
                        constraints.push_back(Constraint{static_cast<int>(j), static_cast<int>(t + 1), {path[t+1], path[t]}});
                    }
                }
            }
        }

        // Path publish (각 포즈에 timestep별 3초 간격 시간 부여)
        rclcpp::Time start_time = this->now();
        double dt = 3.0; // 3초 간격

        for (size_t i = 0; i < agent_num_; ++i) {
            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp = start_time;
            path_msg.header.frame_id = "map";
            for (size_t t = 0; t < planned_paths[i].size(); ++t) {
                const auto& p = planned_paths[i][t];
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                // 각 포즈에 "시작시각 + 3초 × timestep"을 넣는다
                pose.header.stamp = start_time + rclcpp::Duration::from_seconds(dt * t);
                pose.pose.position.x = msg->info.origin.position.x + (p.second + 0.5f) * msg->info.resolution;
                pose.pose.position.y = msg->info.origin.position.y + (p.first + 0.5f) * msg->info.resolution;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;
                path_msg.poses.push_back(pose);
            }
            path_pubs_[i]->publish(path_msg);
            RCLCPP_INFO(this->get_logger(), "Publishing /waypoints%ld: %ld poses", i+1, path_msg.poses.size());
        }
    }

    // 클래스 멤버
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    std::vector<std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>>> path_pubs_;
    std::vector<Position> agent_starts_, agent_goals_;
    size_t agent_num_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PrioritizedPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
