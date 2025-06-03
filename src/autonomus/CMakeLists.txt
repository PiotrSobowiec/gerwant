cmake_minimum_required(VERSION 3.5)
project(autonomus)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(face_marker_tracker REQUIRED)   # żeby używać VisionResult
find_package(dynamic_reconfigure REQUIRED)

# === dynamic_reconfigure ===
generate_dynamic_reconfigure_options(
  cfg/Autonomous.cfg
)
# =========================

include_directories(include)

add_executable(autonomous_node src/autonomous_node.cpp)
ament_target_dependencies(autonomous_node
  rclcpp
  std_msgs
  geometry_msgs
  face_marker_tracker
)
add_dependencies(autonomous_node ${PROJECT_NAME}_gencfg)

ament_package(
  DEPENDS
    rclcpp
    std_msgs
    geometry_msgs
    face_marker_tracker
    dynamic_reconfigure
)
