// autonomous_node.cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Vector3.h>
#include <dynamic_reconfigure/server.h>
#include <autonomous/AutonomousConfig.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <filesystem>
#include <fstream>

#include "autonomus/autonomous.hpp"
#include "face_marker_tracker/vision.hpp"

using autonomous::AutonomousConfig;

// Enhanced state machine for autonomous behavior
enum State {
    SEARCH_FACE,
    FOLLOW_FACE,
    SEARCH_MARKER,
    FOLLOW_MARKER,
    AVOID_OBSTACLE
};

class AutonomousNode {
public:
    AutonomousNode(ros::NodeHandle& nh)
      : nh_(nh)
      , hasVision_(false)
      , hasSlamPose_(false)
      , frontDist_(std::numeric_limits<float>::infinity())
      , flDist_(std::numeric_limits<float>::infinity())
      , frDist_(std::numeric_limits<float>::infinity())
      , rearDist_(std::numeric_limits<float>::infinity())
      , state_(SEARCH_FACE)
    {}

    bool initialize() {
        // Static parameters
        nh_.param<std::string>("~vision_topic", visionTopic_, "/vision/result");
        nh_.param<std::string>("~sensor_front_topic", sensorFrontTopic_, "/sensors/front");
        nh_.param<std::string>("~sensor_fl_topic", sensorFLTopic_, "/sensors/front_left");
        nh_.param<std::string>("~sensor_fr_topic", sensorFRTopic_, "/sensors/front_right");
        nh_.param<std::string>("~sensor_rear_topic", sensorRearTopic_, "/sensors/rear");
        nh_.param<std::string>("~cmd_vel_topic", cmdVelTopic_, "/cmd_vel");
        nh_.param<std::string>("~slam_pose_topic", slamPoseTopic_, "/orb_slam2/pose");
        nh_.param<int>("~frame_width", frameWidth_, 640);

        // Dynamic reconfigure
        dynamic_reconfigure::Server<AutonomousConfig>::CallbackType cb =
            boost::bind(&AutonomousNode::reconfigureCallback, this, _1, _2);
        server_.setCallback(cb);

        // ROS interfaces
        visionSub_ = nh_.subscribe(visionTopic_, 10, &AutonomousNode::visionCallback, this);
        frontSub_  = nh_.subscribe(sensorFrontTopic_, 10, &AutonomousNode::frontCallback, this);
        flSub_     = nh_.subscribe(sensorFLTopic_,    10, &AutonomousNode::flCallback,    this);
        frSub_     = nh_.subscribe(sensorFRTopic_,    10, &AutonomousNode::frCallback,    this);
        rearSub_   = nh_.subscribe(sensorRearTopic_,  10, &AutonomousNode::rearCallback,  this);
        slamSub_   = nh_.subscribe(slamPoseTopic_,   10, &AutonomousNode::slamCallback,   this);
        cmdPub_    = nh_.advertise<geometry_msgs::Twist>(cmdVelTopic_, 10);

        // Setup logging directory
        fs::path logDir = fs::current_path() / "logs" / "autonomous";
        fs::create_directories(logDir);
        stateLog_.open((logDir / "state_log.csv").string(), std::ios::out);
        slamLog_.open((logDir / "slam_log.csv").string(), std::ios::out);
        if (stateLog_) stateLog_ << "timestamp,state\n";
        if (slamLog_)  slamLog_  << "timestamp,px,py,pz,qx,qy,qz,qw\n";

        ROS_INFO("[AutonomousNode] initialized (frame_width=%d) ", frameWidth_);
        return true;
    }

    void spin() {
        ros::Rate rate(20);
        while (ros::ok()) {
            ros::spinOnce();
            State newState = state_;

            // State transitions
            if (frontDist_ < safeDistance_) {
                newState = AVOID_OBSTACLE;
            } else if (state_ == AVOID_OBSTACLE && frontDist_ >= safeDistance_) {
                newState = SEARCH_FACE;
            } else if (state_ == SEARCH_FACE && hasVision_) {
                newState = FOLLOW_FACE;
            } else if (state_ == FOLLOW_FACE && hasVision_ && lastVision_.status == "MarkerFound") {
                newState = FOLLOW_MARKER;
            } else if (state_ == FOLLOW_MARKER && (!hasVision_ || lastVision_.status != "MarkerFound")) {
                newState = SEARCH_MARKER;
            } else if (state_ == SEARCH_MARKER && hasVision_ && lastVision_.status == "MarkerFound") {
                newState = FOLLOW_MARKER;
            }

            // Log state changes
            if (newState != state_) {
                double t = ros::Time::now().toSec();
                stateLog_ << t << ',' << stateToString(newState) << '\n';
                state_ = newState;
            }

            // Behavior per state
            geometry_msgs::Twist cmd;
            switch (state_) {
                case AVOID_OBSTACLE:
                    cmd.linear.x  = 0;
                    cmd.angular.z = (flDist_ > frDist_ ? maxAngular_ : -maxAngular_);
                    break;
                case SEARCH_FACE:
                    cmd.linear.x  = 0;
                    cmd.angular.z = maxAngular_ / 2.0;
                    break;
                case FOLLOW_FACE:
                    {
                        double cx = lastVision_.faceRect.x + lastVision_.faceRect.width*0.5;
                        double errX = cx - frameWidth_*0.5;
                        cmd.angular.z = -(errX/(frameWidth_*0.5))*maxAngular_;
                        cmd.linear.x  = std::clamp((double)followDistance_/std::max(0.1f,frontDist_), 0.0, maxLinear_);
                    }
                    break;
                case SEARCH_MARKER:
                    cmd.linear.x  = 0;
                    cmd.angular.z = maxAngular_ / 2.0;
                    break;
                case FOLLOW_MARKER:
                    {
                        double errX = lastVision_.markerCenter.x - frameWidth_*0.5;
                        cmd.angular.z = -(errX/(frameWidth_*0.5))*maxAngular_;
                        double errD = frontDist_ - followDistance_;
                        cmd.linear.x  = std::clamp(errD/followDistance_, 0.0, maxLinear_);
                    }
                    break;
            }
            cmdPub_.publish(cmd);

            rate.sleep();
        }
        // Close logs
        if (stateLog_.is_open()) stateLog_.close();
        if (slamLog_.is_open())  slamLog_.close();
    }

private:
    // State to string
    std::string stateToString(State s) {
        switch (s) {
            case SEARCH_FACE:   return "SEARCH_FACE";
            case FOLLOW_FACE:   return "FOLLOW_FACE";
            case SEARCH_MARKER: return "SEARCH_MARKER";
            case FOLLOW_MARKER: return "FOLLOW_MARKER";
            case AVOID_OBSTACLE:return "AVOID_OBSTACLE";
        }
        return "UNKNOWN";
    }

    // Dynamic reconfigure
    void reconfigureCallback(AutonomousConfig &config, uint32_t) {
        followDistance_ = config.follow_distance;
        safeDistance_   = config.safe_distance;
        maxLinear_      = config.max_linear_speed;
        maxAngular_     = config.max_angular_speed;
        ROS_INFO("[Reconfigure] follow=%.1f safe=%.1f lin=%.2f ang=%.2f",
                 followDistance_, safeDistance_, maxLinear_, maxAngular_);
    }

    // Callbacks
    void visionCallback(const vision_msgs::VisionResultConstPtr& msg) {
        lastVision_ = *msg;
        hasVision_  = true;
    }
    void frontCallback(const std_msgs::Float32ConstPtr& msg) { frontDist_ = msg->data; }
    void flCallback   (const std_msgs::Float32ConstPtr& msg) { flDist_    = msg->data; }
    void frCallback   (const std_msgs::Float32ConstPtr& msg) { frDist_    = msg->data; }
    void rearCallback (const std_msgs::Float32ConstPtr& msg) { rearDist_  = msg->data; }
    void slamCallback (const geometry_msgs::PoseStampedConstPtr& msg) {
        hasSlamPose_ = true;
        lastPose_    = msg->pose;
        double t = msg->header.stamp.toSec();
        slamLog_ << t << ','
                 << lastPose_.position.x << ','
                 << lastPose_.position.y << ','
                 << lastPose_.position.z << ','
                 << lastPose_.orientation.x << ','
                 << lastPose_.orientation.y << ','
                 << lastPose_.orientation.z << ','
                 << lastPose_.orientation.w << '\n';
    }

    ros::NodeHandle nh_;
    ros::Subscriber visionSub_, frontSub_, flSub_, frSub_, rearSub_, slamSub_;
    ros::Publisher cmdPub_;
    dynamic_reconfigure::Server<AutonomousConfig> server_;

    std::string visionTopic_, sensorFrontTopic_, sensorFLTopic_, sensorFRTopic_, sensorRearTopic_, cmdVelTopic_, slamPoseTopic_;
    int frameWidth_;

    // Dynamic parameters
    float followDistance_, safeDistance_, maxLinear_, maxAngular_;

    // Vision and sensors
    VisionResult lastVision_;
    bool hasVision_;
    float frontDist_, flDist_, frDist_, rearDist_;

    // SLAM pose
    bool hasSlamPose_;
    geometry_msgs::Pose lastPose_;
    std::ofstream slamLog_;

    // State machine
    State state_;
    std::ofstream stateLog_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "autonomous_node");
    ros::NodeHandle nh;
    AutonomousNode node(nh);
    if (!node.initialize()) return 1;
    node.spin();
    return 0;
}
