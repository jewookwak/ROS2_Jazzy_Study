cmake_minimum_required(VERSION 3.8)
project(traffic_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# TrafficPlanner 알고리즘 라이브러리 (ROS 독립적)
add_library(traffic_planner_lib
  src/TrafficPlanner.cpp
)

# TrafficPlannerNode 라이브러리 (ROS2 의존적)
add_library(traffic_planner_node_lib
  src/TrafficPlannerNode.cpp
)

# 실행 파일
add_executable(traffic_path_planner
  src/main.cpp
)

# 라이브러리 링크
target_link_libraries(traffic_planner_node_lib traffic_planner_lib)
target_link_libraries(traffic_path_planner traffic_planner_node_lib)

# ROS2 의존성 (노드 라이브러리와 실행 파일에만 적용)
ament_target_dependencies(traffic_planner_node_lib
  rclcpp
  geometry_msgs
  nav_msgs
)

ament_target_dependencies(traffic_path_planner
  rclcpp
  geometry_msgs
  nav_msgs
)

# 설치
install(TARGETS 
  traffic_path_planner 
  traffic_planner_lib 
  traffic_planner_node_lib
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 파일 설치 (다른 패키지에서 사용할 수 있도록)
install(DIRECTORY include/
  DESTINATION include/
)

# Launch 파일 설치
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

ament_package()