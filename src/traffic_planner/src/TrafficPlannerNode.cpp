#include "traffic_planner/TrafficPlannerNode.hpp"
#include <chrono>

using namespace std::chrono_literals;

TrafficPlannerNode::TrafficPlannerNode() : Node("traffic_path_planner_node") {
    RCLCPP_INFO(get_logger(), "📌 Traffic Path Planner Node 시작");
    
    initializeParameters();
    initializeMap();
    initializeRobots();
    
    RCLCPP_INFO(get_logger(), "🎉 %d개 로봇에 대한 Traffic Path Planner 초기화 완료", num_robots_);
}

void TrafficPlannerNode::initializeParameters() {
    // 파라미터 선언 및 기본값 설정
    declare_parameter("num_robots", 2);
    declare_parameter("timeout_seconds", 800.0);
    declare_parameter("resolution", 0.1);
    declare_parameter("start_positions", std::vector<double>{});
    declare_parameter("goal_positions", std::vector<double>{});

    num_robots_ = get_parameter("num_robots").as_int();
    timeout_seconds_ = get_parameter("timeout_seconds").as_double();
    resolution_ = get_parameter("resolution").as_double();

    RCLCPP_INFO(get_logger(), "🤖 로봇 수: %d", num_robots_);
    RCLCPP_INFO(get_logger(), "⏰ 타임아웃: %.1f초", timeout_seconds_);
    RCLCPP_INFO(get_logger(), "📏 해상도: %.2fm", resolution_);

    if (num_robots_ <= 0) {
        RCLCPP_ERROR(get_logger(), "❌ 로봇 수는 1 이상이어야 합니다");
        throw std::invalid_argument("로봇 수가 유효하지 않습니다");
    }
}

void TrafficPlannerNode::initializeMap() {
    // 교통 규칙이 적용된 맵 생성
    auto map = createDefaultMap();
    planner_ = std::make_unique<TrafficPlanner>(map);
    RCLCPP_INFO(get_logger(), "🗺️  교통 규칙 맵 초기화 완료");
}

void TrafficPlannerNode::initializeRobots() {
    auto starts = generateStartPositions();
    auto goals = generateGoalPositions();

    if (starts.size() != static_cast<size_t>(num_robots_) || 
        goals.size() != static_cast<size_t>(num_robots_)) {
        RCLCPP_ERROR(get_logger(), "❌ 시작점 또는 목표점 수가 로봇 수와 일치하지 않음");
        throw std::runtime_error("시작점/목표점 개수 불일치");
    }

    // 시작점과 목표점 출력
    for (int i = 0; i < num_robots_; ++i) {
        RCLCPP_INFO(get_logger(), "로봇 %d: 시작(%d,%d) → 목표(%d,%d)", 
                    i+1, starts[i].first, starts[i].second, goals[i].first, goals[i].second);
    }

    // CBS로 경로 계획
    planAndAssignPaths();

    // 각 로봇에 대해 ROS2 통신 설정
    for (int id = 1; id <= num_robots_; ++id) {
        createRobotCommunication(id);
        publishPath(id);
    }
}

void TrafficPlannerNode::planAndAssignPaths() {
    auto starts = generateStartPositions();
    auto goals = generateGoalPositions();

    // std::vector<Constraint> custom_constraints = {
    //     {0, 3, { {5, 10} }},
    //     {1, 4, { {6, 11}, {6, 12} }}
    // };
    std::vector<Constraint> custom_constraints = {
        {0, 3, { {5, 10} }},
    };
    RCLCPP_INFO(get_logger(), "🚀 Traffic-aware CBS 솔버 시작 - 로봇 %d개", num_robots_);
    
    // auto paths = planner_->planPaths(starts, goals, true);  // disjoint splitting 사용
    auto paths = planner_->planPaths(starts, goals, true, custom_constraints);  // custom_constraints 추가


    if (paths.size() != static_cast<size_t>(num_robots_)) {
        RCLCPP_ERROR(get_logger(), "❌ Traffic-aware CBS 경로 생성 실패");
        throw std::runtime_error("경로 생성 실패");
    }

    RCLCPP_INFO(get_logger(), "✅ Traffic-aware CBS 경로 생성 성공! %zu개 경로 생성됨", paths.size());

    // 경로를 각 로봇의 waypoints로 변환
    for (int id = 1; id <= num_robots_; ++id) {
        auto& rd = robots_[id];
        rd.waypoints = convertPathToWaypoints(paths[id - 1]);
        RCLCPP_INFO(get_logger(), "[%d] Traffic-aware waypoints 생성 완료: %zu개", id, rd.waypoints.size());
    }
}

void TrafficPlannerNode::createRobotCommunication(int robot_id) {
    auto& rd = robots_[robot_id];

    // pose subscription
    std::string pose_topic = "/robot" + std::to_string(robot_id) + "/pose";
    rd.pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic, 10,
        [this, robot_id](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            handlePose(robot_id, msg);
        });

    // goal publisher
    std::string goal_topic = "/goalpose" + std::to_string(robot_id);
    rd.goal_pub = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);

    // path publisher
    std::string path_topic = "/path" + std::to_string(robot_id);
    rd.path_pub = create_publisher<nav_msgs::msg::Path>(path_topic, 10);

    RCLCPP_INFO(get_logger(), "[%d] ROS2 통신 설정 완료", robot_id);
}

std::vector<Position> TrafficPlannerNode::generateStartPositions() {
    // 파라미터에서 커스텀 시작점 읽기
    auto custom_starts = get_parameter("start_positions").as_double_array();
    
    if (custom_starts.size() == static_cast<size_t>(num_robots_ * 2)) {
        std::vector<Position> starts;
        for (int i = 0; i < num_robots_; ++i) {
            int row = static_cast<int>(custom_starts[i * 2]);
            int col = static_cast<int>(custom_starts[i * 2 + 1]);
            starts.push_back({row, col});
        }
        RCLCPP_INFO(get_logger(), "📍 커스텀 시작점 사용");
        return starts;
    }

    // 기본 시작점 생성 (교통 흐름을 고려한 배치)
    std::vector<Position> starts;
    int start_row = 2;  // 시작 행
    int start_col = 19; // 우측 가장자리
    
    for (int i = 0; i < num_robots_; ++i) {
        starts.push_back({start_row + i * 2, start_col});
    }
    
    RCLCPP_INFO(get_logger(), "📍 교통 흐름 기반 기본 시작점 생성 완료 (%d개)", num_robots_);
    return starts;
}

std::vector<Position> TrafficPlannerNode::generateGoalPositions() {
    // 파라미터에서 커스텀 목표점 읽기
    auto custom_goals = get_parameter("goal_positions").as_double_array();
    
    if (custom_goals.size() == static_cast<size_t>(num_robots_ * 2)) {
        std::vector<Position> goals;
        for (int i = 0; i < num_robots_; ++i) {
            int row = static_cast<int>(custom_goals[i * 2]);
            int col = static_cast<int>(custom_goals[i * 2 + 1]);
            goals.push_back({row, col});
        }
        RCLCPP_INFO(get_logger(), "🎯 커스텀 목표점 사용");
        return goals;
    }

    // 기본 목표점 생성 (교통 패턴을 고려한 배치)
    std::vector<Position> goals;
    int goal_row = 3;   // 시작 행
    int goal_col = 11;  // 중앙 영역
    
    for (int i = 0; i < num_robots_; ++i) {
        goals.push_back({goal_row + i * 2, goal_col});
    }
    
    RCLCPP_INFO(get_logger(), "🎯 교통 패턴 기반 기본 목표점 생성 완료 (%d개)", num_robots_);
    return goals;
}

std::vector<WaypointWithYaw> TrafficPlannerNode::convertPathToWaypoints(const std::vector<Position>& path) {
    std::vector<WaypointWithYaw> waypoints;
    
    for (const auto& cell : path) {
        double x = (cell.second - 1) * resolution_ + resolution_ / 2.0;
        double y = (cell.first - 1)  * resolution_ + resolution_ / 2.0;
        double yaw = M_PI / 2.0;  // 기본 방향
        waypoints.push_back({x, y, yaw});
    }
    
    return waypoints;
}

void TrafficPlannerNode::publishPath(int id) {
    auto& rd = robots_[id];
    
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = now();

    // waypoints를 Path 메시지로 변환
    for (const auto& wp : rd.waypoints) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now();
        pose.pose.position.x = wp.x;
        pose.pose.position.y = wp.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.z = std::sin(wp.yaw / 2.0);
        pose.pose.orientation.w = std::cos(wp.yaw / 2.0);
        
        path_msg.poses.push_back(pose);
    }

    rd.path_pub->publish(path_msg);
    RCLCPP_INFO(get_logger(), "[%d] 🛤️  교통 규칙 기반 경로 publish 완료 (%zu waypoints)", id, rd.waypoints.size());
}

void TrafficPlannerNode::handlePose(int id, geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    auto& rd = robots_[id];
    rd.current_pose = *msg;

    if (rd.waypoint_index >= rd.waypoints.size()) return;

    const auto& wp = rd.waypoints[rd.waypoint_index];
    double dx = rd.current_pose.pose.position.x - wp.x;
    double dy = rd.current_pose.pose.position.y - wp.y;

    if (!rd.goal_sent) {
        sendGoalPose(id);
        startTimeout(id);
        return;
    }

    if (dx * dx + dy * dy < RobotData::THRESH_SQ) {
        RCLCPP_INFO(get_logger(), "✅ [%d] 교통 규칙 준수하며 wp[%zu] 도달!", id, rd.waypoint_index);
        if (rd.timeout_timer) rd.timeout_timer->cancel();
        advanceWaypoint(id);
    }
}

void TrafficPlannerNode::sendGoalPose(int id) {
    auto& rd = robots_[id];
    const auto& wp = rd.waypoints[rd.waypoint_index];

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
        "[%d] 🟢 교통 규칙 기반 goalpose publish → wp[%zu] (%.2f, %.2f)", 
        id, rd.waypoint_index, wp.x, wp.y);
}

void TrafficPlannerNode::startTimeout(int id) {
    auto& rd = robots_[id];
    if (rd.timeout_timer) rd.timeout_timer->cancel();

    rd.timeout_timer = create_wall_timer(
        std::chrono::duration<double>(timeout_seconds_),
        [this, id]() {
            RCLCPP_WARN(get_logger(), "[%d] ⏰ %.0f초 timeout. 다음 waypoint로 이동", id, timeout_seconds_);
            robots_[id].timeout_timer->cancel();
            advanceWaypoint(id);
        });
}

void TrafficPlannerNode::advanceWaypoint(int id) {
    auto& rd = robots_[id];
    rd.waypoint_index++;
    rd.goal_sent = false;

    if (rd.waypoint_index < rd.waypoints.size()) {
        sendGoalPose(id);
        startTimeout(id);
    } else {
        RCLCPP_INFO(get_logger(), "🎉 [%d] 교통 규칙을 준수하며 모든 waypoint 완료", id);
    }
}

std::vector<std::vector<bool>> TrafficPlannerNode::createDefaultMap() {
    return {
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
    };
}
