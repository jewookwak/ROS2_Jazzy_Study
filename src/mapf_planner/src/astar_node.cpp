#include "rclcpp/rclcpp.hpp"
#include "mapf_planner/astar.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class AStarNode : public rclcpp::Node {
public:
    AStarNode() : Node("astar_node") {
        // /map 구독
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/grid_map", 10,
            std::bind(&AStarNode::mapCallback, this, std::placeholders::_1)
        );

        // /waypoints 퍼블리셔
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/waypoints", 10);
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received /map:");
        RCLCPP_INFO(this->get_logger(), "- width: %d, height: %d", msg->info.width, msg->info.height);
        RCLCPP_INFO(this->get_logger(), "- resolution: %.3f", msg->info.resolution);
        RCLCPP_INFO(this->get_logger(), "- origin: (%.2f, %.2f)",
                    msg->info.origin.position.x, msg->info.origin.position.y);

        // OccupancyGrid → bool 맵
        std::vector<std::vector<bool>> map(msg->info.height, std::vector<bool>(msg->info.width));
        for (size_t y = 0; y < msg->info.height; ++y) {
            for (size_t x = 0; x < msg->info.width; ++x) {
                int idx = y * msg->info.width + x;
                map[y][x] = (msg->data[idx] == 100);
            }
        }

        // 시작/목표점 (테두리 제외)
        int start_y = 1, start_x = 1;
        int goal_y = static_cast<int>(msg->info.height - 2);
        int goal_x = static_cast<int>(msg->info.width - 2);

        RCLCPP_INFO(this->get_logger(), "- Start: (%d, %d), Goal: (%d, %d)", start_y, start_x, goal_y, goal_x);

        // 맨해튼 거리 휴리스틱
        std::vector<std::vector<int>> heuristics(msg->info.height, std::vector<int>(msg->info.width, 0));
        for (size_t y = 0; y < msg->info.height; ++y) {
            for (size_t x = 0; x < msg->info.width; ++x) {
                heuristics[y][x] = std::abs(goal_y - static_cast<int>(y)) + std::abs(goal_x - static_cast<int>(x));
            }
        }

        // A* 경로 생성
        AStarPlanner planner(map);
        auto path = planner.findPath({start_y, start_x}, {goal_y, goal_x}, heuristics, 1, {});

        // nav_msgs/Path 메시지 생성
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "map";

        for (const auto& p : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = msg->info.origin.position.x + (p.second + 0.5f) * msg->info.resolution;
            pose.pose.position.y = msg->info.origin.position.y + (p.first  + 0.5f) * msg->info.resolution;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        // publish
        path_publisher_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Published path with %ld waypoints to /waypoints", path.size());

        // Path 디버그 출력
        std::ostringstream oss;
        oss << "Path: ";
        for (const auto& p : path) {
            oss << "(" << p.first << "," << p.second << ") ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarNode>());
    rclcpp::shutdown();
    return 0;
}
