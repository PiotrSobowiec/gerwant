// sensors.hpp
#pragma once

#include <ros/ros.h>
#include <vector>

// ROS parameters (~ namespace)
// trig_pins: list<int> length 4
// echo_pins: list<int> length 4
// imu_i2c_address: int (domyślnie 0x68)
// read_rate: double (Hz)

// Pozycja czujnika na pojeździe
enum SensorPosition { FRONT=0, FRONT_LEFT=1, FRONT_RIGHT=2, REAR=3 };

class SensorsNode {
public:
    SensorsNode(ros::NodeHandle& nh);
    bool initialize(); // wczytuje parametry, inicjalizuje hardware
    void spin();       // petla odczytów i publikacji
private:
    void loadParameters();
    void readSensors();

    ros::NodeHandle nh_;
    std::vector<UltrasonicSensor> ultrasonicSensors_;
    IMUSensor imuSensor_;
    ros::Publisher distPub_[4];       // front, fl, fr, rear
    ros::Publisher imuPub_;           // geometry_msgs/Vector3
    double readRate_;
};
