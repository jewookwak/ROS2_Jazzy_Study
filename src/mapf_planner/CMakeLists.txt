cmake_minimum_required(VERSION 3.8)
project(mapf_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# include 디렉토리 설정
include_directories(include)

# === 실행 파일 설정 ===
add_executable(astar_node
  src/astar_node.cpp
  src/astar.cpp
)

add_executable(grid_map_publisher
  src/grid_map_publisher.cpp
)

add_executable(prioritized_planner_node
  src/prioritized_planner_node.cpp
  src/astar.cpp
)

add_executable(cbs_planner_node
  src/cbs_planner_node.cpp
  src/cbs.cpp
  src/astar.cpp
)

add_executable(waypoint_follower 
  src/waypoint_follower.cpp
)

add_executable(tracked_path_publisher 
  src/tracked_path_publisher.cpp
)

add_executable(cbs_planner_node_test
  src/cbs_planner_node_test.cpp
  src/cbs.cpp
  src/astar.cpp
)

add_executable(cbs_planner_node_test2
  src/cbs_planner_node_test2.cpp
  src/cbs.cpp
  src/astar.cpp
)

add_executable(cbs_planner_pose_publisher
  src/cbs_planner_pose_publisher.cpp
  src/cbs.cpp
  src/astar.cpp
)

add_executable(cbs_planner_node_pose_pub
  src/cbs_planner_node_pose_pub.cpp
  src/cbs.cpp
  src/astar.cpp
)

# === 의존성 연결 ===
ament_target_dependencies(astar_node
  rclcpp
  rclcpp_action
  nav_msgs
  std_msgs
  geometry_msgs
  nav2_msgs
)

ament_target_dependencies(grid_map_publisher
  rclcpp
  nav_msgs
  geometry_msgs
)

ament_target_dependencies(prioritized_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
)

ament_target_dependencies(cbs_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
)

ament_target_dependencies(waypoint_follower
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

ament_target_dependencies(tracked_path_publisher 
  rclcpp 
  geometry_msgs 
  nav_msgs
)

ament_target_dependencies(cbs_planner_node_test
  rclcpp
  nav_msgs
  geometry_msgs
  nav2_msgs
  rclcpp_action
  visualization_msgs
)

ament_target_dependencies(cbs_planner_node_test2
  rclcpp
  nav_msgs
  geometry_msgs
  nav2_msgs
  rclcpp_action
  visualization_msgs
)

ament_target_dependencies(cbs_planner_pose_publisher
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

ament_target_dependencies(cbs_planner_node_pose_pub
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

# include 디렉토리 설정
target_include_directories(astar_node PUBLIC include)
target_include_directories(prioritized_planner_node PUBLIC include)
target_include_directories(cbs_planner_node PUBLIC include)
target_include_directories(cbs_planner_node_test PUBLIC include)
target_include_directories(cbs_planner_node_test2 PUBLIC include)
target_include_directories(cbs_planner_pose_publisher PUBLIC include)
target_include_directories(cbs_planner_node_pose_pub PUBLIC include)

# === 설치 설정 ===
install(TARGETS
  astar_node
  grid_map_publisher
  prioritized_planner_node
  cbs_planner_node
  waypoint_follower
  tracked_path_publisher
  cbs_planner_node_test
  cbs_planner_node_test2
  cbs_planner_pose_publisher
  cbs_planner_node_pose_pub
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()