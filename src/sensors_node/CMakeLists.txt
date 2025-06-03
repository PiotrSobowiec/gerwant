cmake_minimum_required(VERSION 3.5)
project(sensors_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

include_directories(include)

add_executable(sensors_node
  src/sensors_node.cpp
  src/sensors.cpp
)
ament_target_dependencies(sensors_node
  rclcpp
  std_msgs
  sensor_msgs
  diagnostic_msgs
)

install(TARGETS sensors_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
