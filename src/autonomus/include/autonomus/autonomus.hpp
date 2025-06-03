#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Vector3.h>
#include <dynamic_reconfigure/server.h>
#include <autonomous/AutonomousConfig.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <filesystem>

#include "face_marker_tracker/vision.hpp"

namespace fs = std::filesystem;

// Possible states for the autonomous state machine
enum State {
    SEARCH_FACE,
    FOLLOW_FACE,
    SEARCH_MARKER,
    FOLLOW_MARKER,
    AVOID_OBSTACLE
};

class AutonomousNode {
public:
    explicit AutonomousNode(ros::NodeHandle& nh);
    bool initialize();      // Load parameters, setup ROS interfaces, and prepare logs
    void spin();           // Main loop: state transitions and behavior execution

private:
    void reconfigureCallback(autonomous::AutonomousConfig &config, uint32_t level);
    void visionCallback(const vision_msgs::VisionResultConstPtr& msg);
    void frontCallback(const std_msgs::Float32ConstPtr& msg);
    void flCallback(const std_msgs::Float32ConstPtr& msg);
    void frCallback(const std_msgs::Float32ConstPtr& msg);
    void rearCallback(const std_msgs::Float32ConstPtr& msg);
    void slamCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    std::string stateToString(State s);

    ros::NodeHandle nh_;
    ros::Subscriber visionSub_, frontSub_, flSub_, frSub_, rearSub_, slamSub_;
    ros::Publisher cmdPub_;
    dynamic_reconfigure::Server<autonomous::AutonomousConfig> server_;

    // Topic names and frame width
    std::string visionTopic_, sensorFrontTopic_, sensorFLTopic_, sensorFRTopic_, sensorRearTopic_, cmdVelTopic_, slamPoseTopic_;
    int frameWidth_;

    // Dynamic parameters
    float followDistance_, safeDistance_, maxLinear_, maxAngular_;

    // Sensor and vision data
    vision::VisionResult lastVision_;  // Ensure VisionResult is in namespace vision or adjust include
    bool hasVision_;
    float frontDist_, flDist_, frDist_, rearDist_;

    // SLAM pose data
    bool hasSlamPose_;
    geometry_msgs::Pose lastPose_;

    // State machine and logging
    State state_;
    std::ofstream stateLog_;
    std::ofstream slamLog_;
};
