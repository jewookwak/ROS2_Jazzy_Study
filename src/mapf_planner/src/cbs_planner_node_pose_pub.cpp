#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mapf_planner/cbs.hpp"

#include <vector>
#include <cmath>
#include <map>
#include <string>

using namespace std::chrono_literals;

struct WaypointWithYaw {
  double x, y, yaw;
};

struct RobotData {
  std::vector<WaypointWithYaw> waypoints;
  size_t waypoint_index{0};
  geometry_msgs::msg::PoseStamped current_pose;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
  rclcpp::TimerBase::SharedPtr timeout_timer;
  bool goal_sent{false};

  static constexpr double THRESHOLD = 0.04;
  static constexpr double THRESH_SQ = THRESHOLD * THRESHOLD;
};

class CBSPlannerNode : public rclcpp::Node {
public:
  CBSPlannerNode() : Node("cbs_planner_node_pose_pub") {
    RCLCPP_INFO(get_logger(), "📌 CBS Planner Pose Publisher Started");

    std::vector<std::vector<bool>> map = {
      {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
      {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
    };

    std::vector<Position> starts = {{2,19},{4,19},{6,19}};
    std::vector<Position> goals  = {{3,11},{5,11},{7,11}};
    CBSSolver solver(map, starts, goals);
    auto paths = solver.findSolution(true);

    if (paths.size() < 3) {
      RCLCPP_ERROR(get_logger(), "❌ CBS 경로 생성 실패 또는 일부 로봇 경로 없음");
      return;
    }

    constexpr double RES = 0.1;
    for (int id = 1; id <= 3; ++id) {
      auto &rd = robots_[id];

      for (const auto &cell : paths[id - 1]) {
        double x = (cell.second - 1) * RES + RES / 2.0;
        double y = (cell.first - 1)  * RES + RES / 2.0;
        double yaw = M_PI / 2.0;
        rd.waypoints.push_back({x, y, yaw});
      }

      std::string pose_topic = "/robot" + std::to_string(id) + "/pose";
      rd.pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic, 10,
        [this, id](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          handle_pose(id, msg);
        });

      std::string goal_topic = "/goalpose" + std::to_string(id);
      rd.goal_pub = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);
    }
  }

private:
  void handle_pose(int id, geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    auto &rd = robots_[id];
    rd.current_pose = *msg;

    if (rd.waypoint_index >= rd.waypoints.size()) return;

    const auto &wp = rd.waypoints[rd.waypoint_index];
    double dx = rd.current_pose.pose.position.x - wp.x;
    double dy = rd.current_pose.pose.position.y - wp.y;

    RCLCPP_INFO(get_logger(),
      "[%d] 현재 위치: (%.2f, %.2f) → 목표: (%.2f, %.2f)",
      id, rd.current_pose.pose.position.x, rd.current_pose.pose.position.y, wp.x, wp.y);

    if (!rd.goal_sent) {
      send_goal_pose(id);
      start_timeout(id);
      return;
    }

    if (dx * dx + dy * dy < RobotData::THRESH_SQ) {
      RCLCPP_INFO(get_logger(), "✅ [%d] wp[%zu] 도달!", id, rd.waypoint_index);
      if (rd.timeout_timer) rd.timeout_timer->cancel();
      advance_waypoint(id);
    }
  }

  void send_goal_pose(int id) {
    auto &rd = robots_[id];
    const auto &wp = rd.waypoints[rd.waypoint_index];

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = now();
    goal.pose.position.x = wp.x;
    goal.pose.position.y = wp.y;
    goal.pose.orientation.z = std::sin(wp.yaw / 2.0);
    goal.pose.orientation.w = std::cos(wp.yaw / 2.0);

    rd.goal_pub->publish(goal);
    rd.goal_sent = true;

    RCLCPP_INFO(get_logger(),
      "[%d] 🟢 goalpose publish → wp[%zu] (%.2f, %.2f)", id, rd.waypoint_index, wp.x, wp.y);
  }

  void start_timeout(int id) {
    auto &rd = robots_[id];
    if (rd.timeout_timer) rd.timeout_timer->cancel();

    rd.timeout_timer = create_wall_timer(
      1000s,
      [this, id]() {
        RCLCPP_WARN(get_logger(), "[%d] ⏰ 1000초 timeout. 다음 waypoint로 이동", id);
        robots_[id].timeout_timer->cancel();
        advance_waypoint(id);
      });
  }

  void advance_waypoint(int id) {
    auto &rd = robots_[id];
    rd.waypoint_index++;
    rd.goal_sent = false;

    if (rd.waypoint_index < rd.waypoints.size()) {
      send_goal_pose(id);
      start_timeout(id);
    } else {
      RCLCPP_INFO(get_logger(), "🎉 [%d] 모든 waypoint 완료", id);
    }
  }

  std::map<int, RobotData> robots_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CBSPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
