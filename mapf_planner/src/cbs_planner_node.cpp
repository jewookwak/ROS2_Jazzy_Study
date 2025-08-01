#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mapf_planner/cbs.hpp"

class CBSPlannerNode : public rclcpp::Node {
public:
    CBSPlannerNode() : Node("cbs_planner_node") {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/multi_agent_path", 10);
        plan();
    }

private:
    void plan() {
        // === 하드코딩된 22x12 맵 ===
        // map[y][x] = 1은 벽, 0은 통로
        std::vector<std::vector<bool>> map = {
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

        // === 시작점과 도착점 설정 ===
        std::vector<Position> starts = {{1, 20}, {10, 1}, {10, 20}};
        std::vector<Position> goals  = {{5, 11}, {10, 20}, {1, 1}};

        // === CBS 실행 ===
        CBSSolver solver(map, starts, goals);
        auto paths = solver.findSolution(true);

        if (paths.empty()) {
            RCLCPP_WARN(this->get_logger(), "No solution found");
            return;
        }

        // === 각 agent 경로 publish 및 출력 ===
        for (size_t i = 0; i < paths.size(); ++i) {
            nav_msgs::msg::Path path_msg;
            path_msg.header.frame_id = "map";
            path_msg.header.stamp = now();
            std::stringstream ss;
            ss << "Agent " << i << " Path: ";

            for (size_t t = 0; t < paths[i].size(); ++t) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path_msg.header;
                pose.pose.position.x = (paths[i][t].second + 0.5) * 0.1;
                pose.pose.position.y = (paths[i][t].first + 0.5) * 0.1;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;

                path_msg.poses.push_back(pose);
                ss << "(" << paths[i][t].second << "," << paths[i][t].first << "," << t << ") ";
            }

            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
            path_pub_->publish(path_msg);
        }
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CBSPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
