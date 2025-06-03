#pragma once

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/face.hpp>

#include <filesystem>
#include <string>
#include <vector>
#include <chrono>
#include <fstream>

// Definicja wyjściowej struktury, niezależnie od ROS
struct VisionResult {
  int frameNumber;
  float fps;
  std::string status;
  std::string personName;
  float confidence;
  cv::Rect faceRect;
  cv::Point2f markerCenter;
};

using ResultCallback = std::function<void(const VisionResult&)>;

class VisionSystem {
public:
  explicit VisionSystem(rclcpp::Node::SharedPtr node);
  bool initialize(int model_choice, int tracker_choice);
  void registerCallback(ResultCallback cb);
  void spinOnce();
  void shutdown();

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void loadParameters();

  rclcpp::Node::SharedPtr node_;
  image_transport::Subscriber imgSub_;
  image_transport::Publisher imgPub_;  // jeśli np. chcesz publikować z nałożonymi ramkami
  ResultCallback callback_;

  int model_choice;
  int tracker_choice;
  std::string cameraTopic_;

  cv::CascadeClassifier face_cascade;
  cv::Ptr<cv::face::FaceRecognizer> face_model;
  std::map<int, std::string> label_names;
  cv::dnn::Net ssd_net;

  cv::Ptr<cv::Tracker> tracker;
  cv::Ptr<cv::aruco::Dictionary> dictionary;

  bool face_verified;
  bool is_tracking;
  int confirmed_label;
  std::string recognized_name;
  float last_confidence;

  cv::Rect target_box;

  // logowanie
  std::ofstream log_file_;
  int frame_counter_;
  double last_fps_;
  int test_id_;
  std::chrono::steady_clock::time_point last_time_;

  // inne pola np. timeouty, czasy wykonania
};
