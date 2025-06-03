cmake_minimum_required(VERSION 3.5)
project(face_marker_tracker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# === Generowanie wiadomości VisionResult ===
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VisionResult.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
# ===========================================

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# === kompilacja węzła wizji ===
add_executable(vision_node
  src/vision_node.cpp
  src/vision.cpp
)
ament_target_dependencies(vision_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)
rosidl_target_interfaces(vision_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
# =================================

ament_export_include_directories(include)
ament_export_libraries(${OpenCV_LIBS})
ament_package()
