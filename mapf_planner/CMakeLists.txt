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

# include 디렉토리 설정
include_directories(
  include
)

# 실행 파일 설정
add_executable(astar_node src/astar_node.cpp src/astar.cpp)
add_executable(grid_map_publisher src/grid_map_publisher.cpp)
add_executable(prioritized_planner_node src/prioritized_planner_node.cpp src/astar.cpp)
ament_target_dependencies(prioritized_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
)
# 타겟에 의존성 연결
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
)

# 설치 설정
install(TARGETS
  astar_node
  grid_map_publisher
  prioritized_planner_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
