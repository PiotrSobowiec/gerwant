#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/vector3.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <vector>
#include <fstream>

// Pozycja czujnika na pojeździe
enum SensorPosition { FRONT = 0, FRONT_LEFT = 1, FRONT_RIGHT = 2, REAR = 3 };

// Forward‐deklaracje klas do obsługi hardware’u
class UltrasonicSensor; // np. w UltrasonicSensor.hpp
class IMUSensor;        // np. w IMUSensor.hpp

class SensorsNode : public rclcpp::Node {
public:
  explicit SensorsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  bool initialize();
  void spin();

private:
  void loadParameters();
  void readSensors();

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distPub_[4];
  rclcpp::Publisher<sensor_msgs::msg::Vector3>::SharedPtr imuPub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagPub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<UltrasonicSensor> ultrasonicSensors_;
  IMUSensor imuSensor_;
  
  double readRate_;
  std::ofstream distCsv_;
  std::ofstream imuCsv_;
};
