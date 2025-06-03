#include <sensor_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <fstream>
#include <filesystem>

#include "sensors_node/sensors.hpp"

namespace fs = std::filesystem;

SensorsNode::SensorsNode(ros::NodeHandle& nh) : nh_(nh) {}

void SensorsNode::loadParameters() {
    // Load ultrasonic sensor pins
    std::vector<int> trigPins, echoPins;
    nh_.param<std::vector<int>>("~trig_pins", trigPins, std::vector<int>{17, 27, 22, 5});
    nh_.param<std::vector<int>>("~echo_pins", echoPins, std::vector<int>{18, 23, 24, 6});
    int imuAddress;
    nh_.param<int>("~imu_i2c_address", imuAddress, 0x68);
    nh_.param<double>("~read_rate", readRate_, 10.0);

    // Initialize UltrasonicSensor objects
    ultrasonicSensors_.clear();
    if (trigPins.size() == 4 && echoPins.size() == 4) {
        for (int i = 0; i < 4; ++i) {
            ultrasonicSensors_.emplace_back(trigPins[i], echoPins[i], static_cast<SensorPosition>(i));
        }
    } else {
        ROS_WARN("Expected 4 trig and echo pins, got %lu and %lu", trigPins.size(), echoPins.size());
    }
}

bool SensorsNode::initialize() {
    loadParameters();
    
    // Initialize IMU
    if (!imuSensor_.begin()) {
        ROS_ERROR("Failed to initialize IMU sensor");
        return false;
    }

    // Advertise topics
    std::string topicBase = "/sensors";
    distPub_[FRONT]       = nh_.advertise<std_msgs::Float32>(topicBase + "/front", 10);
    distPub_[FRONT_LEFT]  = nh_.advertise<std_msgs::Float32>(topicBase + "/front_left", 10);
    distPub_[FRONT_RIGHT] = nh_.advertise<std_msgs::Float32>(topicBase + "/front_right", 10);
    distPub_[REAR]        = nh_.advertise<std_msgs::Float32>(topicBase + "/rear", 10);
    imuPub_               = nh_.advertise<sensor_msgs::Vector3>(topicBase + "/imu", 10);
    diagPub_              = nh_.advertise<diagnostic_msgs::DiagnosticArray>(topicBase + "/diagnostics", 1);

    // Create logs directory
    fs::path logDir = fs::current_path() / "logs" / "sensors";
    if (!fs::exists(logDir)) {
        fs::create_directories(logDir);
    }
    // Open CSV files
    std::string distFile = (logDir / "distances.csv").string();
    std::string imuFile = (logDir / "imu.csv").string();
    distCsv_.open(distFile, std::ios::out);
    imuCsv_.open(imuFile, std::ios::out);
    if (distCsv_ && imuCsv_) {
        // Write headers
        distCsv_ << "timestamp,front,front_left,front_right,rear
";
        imuCsv_  << "timestamp,angleX,angleY,angleZ
";
    } else {
        ROS_WARN("Failed to open log files in %s", logDir.c_str());
    }

    ROS_INFO("SensorsNode initialized");
    return true;
}

void SensorsNode::readSensors() {
    ros::Time now = ros::Time::now();
    diagnostic_msgs::DiagnosticArray diagArray;
    diagArray.header.stamp = now;

    // For CSV logging
    distCsv_ << now << ',';
    
    // Read distances
    std_msgs::Float32 distMsg;
    for (size_t i = 0; i < ultrasonicSensors_.size(); ++i) {
        float d = ultrasonicSensors_[i].readDistance();
        distMsg.data = d;
        distPub_[i].publish(distMsg);
        distCsv_ << d << (i + 1 < ultrasonicSensors_.size() ? ',' : '
');

        diagnostic_msgs::DiagnosticStatus status;
        status.name = "UltrasonicSensor_" + std::to_string(i);
        status.hardware_id = "HC-SR04_" + std::to_string(i);
        if (d <= 0 || d > 200) {
            status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
            status.message = "Invalid distance reading: " + std::to_string(d);
        } else {
            status.level = diagnostic_msgs::DiagnosticStatus::OK;
            status.message = "Distance OK: " + std::to_string(d);
        }
        status.values.clear();
        diagArray.status.push_back(status);
    }
    
    // Read IMU
    imuSensor_.update();
    sensor_msgs::Vector3 imuMsg;
    imuMsg.x = imuSensor_.getAngleX();
    imuMsg.y = imuSensor_.getAngleY();
    imuMsg.z = imuSensor_.getAngleZ();
    imuPub_.publish(imuMsg);

    imuCsv_ << now << ',' << imuMsg.x << ',' << imuMsg.y << ',' << imuMsg.z << '
';

    diagnostic_msgs::DiagnosticStatus imuStatus;
    imuStatus.name = "IMUSensor";
    imuStatus.hardware_id = "MPU6050";
    if (std::isnan(imuMsg.x) || std::isnan(imuMsg.y)) {
        imuStatus.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        imuStatus.message = "IMU returned NaN";
    } else {
        imuStatus.level = diagnostic_msgs::DiagnosticStatus::OK;
        imuStatus.message = "IMU OK";
    }
    diagArray.status.push_back(imuStatus);

    // Publish diagnostics
    diagPub_.publish(diagArray);
}

void SensorsNode::spin() {
    ros::Rate rate(readRate_);
    while (ros::ok()) {
        readSensors();
        ros::spinOnce();
        rate.sleep();
    }
    // Close CSV files on shutdown
    if (distCsv_.is_open()) distCsv_.close();
    if (imuCsv_.is_open()) imuCsv_.close();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensors_node");
    ros::NodeHandle nh;
    SensorsNode node(nh);
    if (!node.initialize()) {
        ROS_FATAL("SensorsNode failed to initialize. Shutting down.");
        return 1;
    }
    node.spin();
    return 0;
}
