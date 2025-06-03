// motion.hpp
#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// ROS parameters (~ namespace)
// wheel_pins: list<int> length 12 (4 motors*3)
// servo_pin: int
// cmd_vel_topic: string (domyślnie "/cmd_vel")
// control_rate: double (Hz)

class MotionNode {
public:
    MotionNode(ros::NodeHandle& nh);
    bool initialize(); // wczytuje parametry oraz wiringPi
    void spin();       // subskrypcja cmd_vel i sterowanie silnikami

private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void loadParameters();

    ros::NodeHandle nh_;
    ros::Subscriber cmdVelSub_;
    std::vector<Motor> motors_;  // 4 silniki
    ServoController servo_;
    std::string cmdVelTopic_;
    double controlRate_;
};
