#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <filesystem>
#include <fstream>
#include <limits>

// Typ stanów
enum State {
  SEARCH_FACE = 0,
  FOLLOW_FACE = 1,
  SEARCH_MARKER = 2,
  FOLLOW_MARKER = 3,
  AVOID_OBSTACLE = 4
};

class AutonomousNode : public rclcpp::Node {
public:
  explicit AutonomousNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  bool initialize();
  void spin();

private:
  // Callback przy zmianie parametrów
  rcl_interfaces::msg::SetParametersResult on_param_change(
    const std::vector<rclcpp::Parameter> & params);

  // subskrypcje
  void vision_callback(const face_marker_tracker::msg::VisionResult::SharedPtr msg);
  void front_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void fl_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void fr_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void rear_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void slam_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // pomocnicze
  std::string state_to_string(State s) const;

  rclcpp::Subscription<face_marker_tracker::msg::VisionResult>::SharedPtr vision_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr front_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fl_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fr_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rear_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slam_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // tematy
  std::string vision_topic_;
  std::string sensor_front_topic_;
  std::string sensor_fl_topic_;
  std::string sensor_fr_topic_;
  std::string sensor_rear_topic_;
  std::string cmd_vel_topic_;
  std::string slam_pose_topic_;
  int frame_width_;

  // parametry modyfikowalne w locie
  float follow_distance_;
  float safe_distance_;
  float max_linear_;
  float max_angular_;

  // stan czujników i wizji
  face_marker_tracker::msg::VisionResult last_vision_;
  bool has_vision_;
  float front_dist_, fl_dist_, fr_dist_, rear_dist_;

  // stan SLAM
  bool has_slam_pose_;
  geometry_msgs::msg::Pose last_pose_;
  std::ofstream slam_log_;

  // maszyna stanów
  State state_;
  std::ofstream state_log_;
};
