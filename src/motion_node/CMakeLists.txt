cmake_minimum_required(VERSION 3.5)
project(motion_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

include_directories(
  include
)

add_executable(motion_node
  src/motion_node.cpp
  src/motion.cpp
)
ament_target_dependencies(motion_node
  rclcpp
  geometry_msgs
  std_srvs
  diagnostic_msgs
)

install(TARGETS motion_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
