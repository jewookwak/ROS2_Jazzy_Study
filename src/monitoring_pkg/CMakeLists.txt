cmake_minimum_required(VERSION 3.8)
project(monitoring_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 필수
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt5 설정
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ROS2 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rcl_interfaces REQUIRED)

# OpenCV 의존성
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

# Qt5 의존성
find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
  Gui
)

# 헤더 파일 포함 디렉토리
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 실행 파일 생성
add_executable(integrated_aruco_monitor
  src/monitoring_gui.cpp
)

# 의존성 라이브러리 링크
ament_target_dependencies(integrated_aruco_monitor
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  rcl_interfaces
)

target_link_libraries(integrated_aruco_monitor
  ${OpenCV_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
)

# 설치 설정
install(TARGETS
  integrated_aruco_monitor
  DESTINATION lib/${PROJECT_NAME}
)

# 런치 파일 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 설정 파일 설치
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()