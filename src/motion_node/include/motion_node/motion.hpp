// motion.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <vector>
#include <string>
#include <fstream>

class Motor;           // zadeklaruj lub do#include "Motor.hpp"
class ServoController; // podobnie

class MotionNode : public rclcpp::Node {
public:
    explicit MotionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    bool initialize();
    void spin();

private:
  void loadParameters();
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void emergencyStopCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void stopMotors();

  // --- pola ROS 2 ---
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagPub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergencySrv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- pola sprzętowe i wewnętrzne ---
  std::vector<Motor> motors_;
  ServoController servo_;

  // --- parametry (wczytane z ROS 2‐owych parametrów) ---
  std::string cmdVelTopic_;
  double controlRate_;
  double cmdTimeout_;
  double maxSpeed_;
  double maxAccel_;

  // --- stan wewnętrzny ---
  double lastCmdTime_;
  bool emergencyStop_;
  double desiredLinear_;
  double desiredAngular_;
  double currentLinear_;

  // --- logowanie do CSV ---
  std::ofstream cmdCsv_;
  std::ofstream stateCsv_;
};