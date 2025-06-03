cmake_minimum_required(VERSION 3.5)
project(face_marker_tracker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# === Generujemy wiadomość VisionResult ===
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VisionResult.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
# ==========================================

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(vision_node
  src/vision_node.cpp
  src/vision.cpp
)
ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  image_transport
  OpenCV
)
rosidl_target_interfaces(vision_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS vision_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
