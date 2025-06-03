// sensors_node.cpp
#include "sensors_node/sensors.hpp"

SensorsNode::SensorsNode(const rclcpp::NodeOptions & options)
: Node("sensors_node", options)
{}

bool SensorsNode::initialize() {
  loadParameters();

  // 1) Inicjalizacja IMU
  if (!imuSensor_.begin()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize IMU sensor");
    return false;
  }

  // 2) Tworzymy publisher’y danych
  std::string topicBase = "/sensors";
  distPub_[FRONT]       = this->create_publisher<std_msgs::msg::Float32>(topicBase + "/front", 10);
  distPub_[FRONT_LEFT]  = this->create_publisher<std_msgs::msg::Float32>(topicBase + "/front_left", 10);
  distPub_[FRONT_RIGHT] = this->create_publisher<std_msgs::msg::Float32>(topicBase + "/front_right", 10);
  distPub_[REAR]        = this->create_publisher<std_msgs::msg::Float32>(topicBase + "/rear", 10);

  imuPub_  = this->create_publisher<sensor_msgs::msg::Vector3>(topicBase + "/imu", 10);
  diagPub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(topicBase + "/diagnostics", 1);

  // 3) Przygotuj katalog logów
  std::filesystem::path logDir = std::filesystem::current_path() / "logs" / "sensors";
  std::filesystem::create_directories(logDir);

  std::string distFile = (logDir / "distances.csv").string();
  std::string imuFile  = (logDir / "imu.csv").string();
  distCsv_.open(distFile);
  imuCsv_.open(imuFile);
  if (distCsv_ && imuCsv_) {
    // poprawnie zakończone literały – używamy '\n' zamiast wielowierszowych
    distCsv_ << "timestamp,front,front_left,front_right,rear\n";
    imuCsv_  << "timestamp,angleX,angleY,angleZ\n";
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to open log files in '%s'",
                logDir.string().c_str());
  }

  // 4) Timer do cyklicznego odczytu sensorów
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(int(1000.0 / readRate_)),
    std::bind(&SensorsNode::readSensors, this)
  );

  RCLCPP_INFO(this->get_logger(), "SensorsNode initialized (read_rate=%.1f Hz)", readRate_);
  return true;
}

void SensorsNode::readSensors() {
  auto now = this->get_clock()->now();

  // 1) Diagnostyka – stwórz array
  diagnostic_msgs::msg::DiagnosticArray diagArray;
  diagArray.header.stamp = now;

  // 2) Pomiar odległości ultradźwiękowych + log/ pub
  std_msgs::msg::Float32 distMsg;
  distCsv_ << now.seconds() << ',';
  for (size_t i = 0; i < ultrasonicSensors_.size(); ++i) {
    float d = ultrasonicSensors_[i].readDistance();
    distMsg.data = d;
    distPub_[i]->publish(distMsg);
    distCsv_ << d << ((i + 1 < ultrasonicSensors_.size()) ? ',' : '\n');

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "UltrasonicSensor_" + std::to_string(i);
    status.hardware_id = "HC-SR04_" + std::to_string(i);
    if (d <= 0 || d > 200) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Invalid distance reading: " + std::to_string(d);
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Distance OK: " + std::to_string(d);
    }
    // w ROS 2 nie ma pola `values` w DiagnosticStatus, więc nie wywołujemy status.values.clear()
    diagArray.status.push_back(status);
  }

  // 3) Pomiar IMU + log/pub
  imuSensor_.update();
  sensor_msgs::msg::Vector3 imuMsg;
  imuMsg.x = imuSensor_.getAngleX();
  imuMsg.y = imuSensor_.getAngleY();
  imuMsg.z = imuSensor_.getAngleZ();
  imuPub_->publish(imuMsg);
  imuCsv_ << now.seconds() << ',' << imuMsg.x << ',' << imuMsg.y << ',' << imuMsg.z << '\n';

  diagnostic_msgs::msg::DiagnosticStatus imuStatus;
  imuStatus.name = "IMUSensor";
  imuStatus.hardware_id = "MPU6050";
  if (std::isnan(imuMsg.x) || std::isnan(imuMsg.y)) {
    imuStatus.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    imuStatus.message = "IMU returned NaN";
  } else {
    imuStatus.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    imuStatus.message = "IMU OK";
  }
  diagArray.status.push_back(imuStatus);

  // 4) publikujemy wszystkie statusy diagnostyczne
  diagPub_->publish(diagArray);
}

void SensorsNode::loadParameters() {
  this->declare_parameter<std::vector<int>>("trig_pins", {17, 27, 22, 5});
  this->declare_parameter<std::vector<int>>("echo_pins", {18, 23, 24, 6});
  this->declare_parameter<int>("imu_i2c_address", 0x68);
  this->declare_parameter<double>("read_rate", 10.0);

  std::vector<int> trigPins, echoPins;
  this->get_parameter("trig_pins", trigPins);
  this->get_parameter("echo_pins", echoPins);
  this->get_parameter("read_rate", readRate_);

  ultrasonicSensors_.clear();
  if (trigPins.size() == 4 && echoPins.size() == 4) {
    for (int i = 0; i < 4; ++i) {
      ultrasonicSensors_.emplace_back(trigPins[i], echoPins[i],
                                      static_cast<SensorPosition>(i));
    }
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Expected 4 trig and 4 echo pins, got %zu and %zu",
                trigPins.size(), echoPins.size());
  }
}

void SensorsNode::spin() {
  // Nic nie robimy tutaj; cały odczyt wykonuje timer
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorsNode>();
  if (!node->initialize()) {
    RCLCPP_FATAL(node->get_logger(),
                 "SensorsNode failed to initialize. Shutting down.");
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
