#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "traffic_planner/TrafficPlanner.hpp"

#include <vector>
#include <cmath>
#include <map>
#include <string>
#include <memory>

struct WaypointWithYaw {
    double x, y, yaw;
};

struct RobotData {
    std::vector<WaypointWithYaw> waypoints;
    size_t waypoint_index{0};
    geometry_msgs::msg::PoseStamped current_pose;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::TimerBase::SharedPtr timeout_timer;
    bool goal_sent{false};

    static constexpr double THRESHOLD = 0.04;
    static constexpr double THRESH_SQ = THRESHOLD * THRESHOLD;
};

class TrafficPlannerNode : public rclcpp::Node {
public:
    TrafficPlannerNode();

private:
    // 핵심 멤버
    std::unique_ptr<TrafficPlanner> planner_;
    std::map<int, RobotData> robots_;
    
    // 파라미터
    int num_robots_;
    double timeout_seconds_;
    double resolution_;
    
    // 초기화 함수들
    void initializeParameters();
    void initializeMap();
    void initializeRobots();
    void createRobotCommunication(int robot_id);
    
    // 시작점/목표점 생성
    std::vector<Position> generateStartPositions();
    std::vector<Position> generateGoalPositions();
    
    // 경로 계획 및 변환
    void planAndAssignPaths();
    std::vector<WaypointWithYaw> convertPathToWaypoints(const std::vector<Position>& path);
    
    // ROS2 통신 함수들
    void handlePose(int id, geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publishPath(int id);
    void sendGoalPose(int id);
    void startTimeout(int id);
    void advanceWaypoint(int id);
    
    // 디폴트 맵 생성
    std::vector<std::vector<bool>> createDefaultMap();
};