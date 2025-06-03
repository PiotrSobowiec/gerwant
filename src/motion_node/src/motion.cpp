#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <std_srvs/SetBool.h>
#include <filesystem>
#include <fstream>

#include "motion_node/motion.hpp"


namespace fs = std::filesystem;

MotionNode::MotionNode(ros::NodeHandle& nh) : nh_(nh) {}

void MotionNode::loadParameters() {
    nh_.param<std::string>("~cmd_vel_topic", cmdVelTopic_, std::string("/cmd_vel"));
    nh_.param<double>("~control_rate", controlRate_, 10.0);
    nh_.param<double>("~cmd_timeout", cmdTimeout_, 1.0);
    nh_.param<double>("~max_speed", maxSpeed_, 1.0);
    nh_.param<double>("~max_accel", maxAccel_, 0.5);

    std::vector<int> wheelPins;
    nh_.param<std::vector<int>>("~wheel_pins", wheelPins, std::vector<int>{2,3,4,5,6,7,8,9,10,11,12,13});
    int servoPin;
    nh_.param<int>("~servo_pin", servoPin, 14);

    motors_.clear();
    if (wheelPins.size() == 12) {
        for (int i = 0; i < 12; i += 3) {
            motors_.emplace_back(wheelPins[i], wheelPins[i+1], wheelPins[i+2]);
        }
    } else {
        ROS_WARN("Expected 12 wheel pins, got %lu", wheelPins.size());
    }

    servo_ = ServoController(servoPin);
}

bool MotionNode::initialize() {
    loadParameters();

    cmdVelSub_ = nh_.subscribe(cmdVelTopic_, 10, &MotionNode::cmdVelCallback, this);
    diagPub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>('/motion/diagnostics', 1);
    emergencySrv_ = nh_.advertiseService("emergency_stop", &MotionNode::emergencyStopCallback, this);

    fs::path logDir = fs::current_path() / "logs" / "motion";
    if (!fs::exists(logDir)) fs::create_directories(logDir);

    std::string cmdFile = (logDir / "commands.csv").string();
    std::string stateFile = (logDir / "state.csv").string();
    cmdCsv_.open(cmdFile, std::ios::out);
    stateCsv_.open(stateFile, std::ios::out);
    if (cmdCsv_) cmdCsv_ << "timestamp,linear_x,angular_z\n";
    if (stateCsv_) stateCsv_ << "timestamp,desired_linear,desired_angular,current_linear,pwm\n";

    lastCmdTime_ = ros::Time::now().toSec();
    emergencyStop_ = false;
    desiredLinear_ = desiredAngular_ = 0.0;
    currentLinear_ = 0.0;

    ROS_INFO("MotionNode initialized, subscribing to %s", cmdVelTopic_.c_str());
    return true;
}

bool MotionNode::emergencyStopCallback(std_srvs::SetBool::Request &req,
                                       std_srvs::SetBool::Response &res) {
    emergencyStop_ = req.data;
    if (emergencyStop_) {
        stopMotors();
        res.message = "Emergency stop engaged";
        res.success = true;
    } else {
        res.message = "Emergency stop released";
        res.success = true;
    }
    return true;
}

void MotionNode::stopMotors() {
    for (auto &motor : motors_) {
        motor.setDirection(STOP);
        motor.setSpeed(0);
    }
    servo_.turn(STRAIGHT);
}

void MotionNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    double now = ros::Time::now().toSec();
    lastCmdTime_ = now;
    desiredLinear_ = std::clamp(msg->linear.x, -maxSpeed_, maxSpeed_);
    desiredAngular_ = msg->angular.z;

    if (cmdCsv_) {
        cmdCsv_ << now << ',' << msg->linear.x << ',' << msg->angular.z << '\n';
    }
}

void MotionNode::spin() {
    ros::Rate rate(controlRate_);
    while (ros::ok()) {
        ros::spinOnce();
        double now = ros::Time::now().toSec();
        if ((now - lastCmdTime_) > cmdTimeout_) {
            desiredLinear_ = 0.0;
            desiredAngular_ = 0.0;
        }

        double delta = maxAccel_ / controlRate_;
        if (currentLinear_ < desiredLinear_) currentLinear_ = std::min(currentLinear_ + delta, desiredLinear_);
        else currentLinear_ = std::max(currentLinear_ - delta, desiredLinear_);

        // Compute PWM value based on currentLinear_
        int pwm = static_cast<int>(std::clamp(currentLinear_ * 100.0, -100.0, 100.0));
        for (auto &motor : motors_) {
            if (emergencyStop_ || pwm == 0) {
                motor.setDirection(STOP);
                motor.setSpeed(0);
            } else if (pwm > 0) {
                motor.setDirection(FORWARD);
                motor.setSpeed(pwm);
            } else {
                motor.setDirection(BACKWARD);
                motor.setSpeed(-pwm);
            }
        }

        if (!emergencyStop_) {
            if (desiredAngular_ > 0.1) servo_.turn(LEFT);
            else if (desiredAngular_ < -0.1) servo_.turn(RIGHT);
            else servo_.turn(STRAIGHT);
        }

        if (stateCsv_) {
            stateCsv_ << now << ',' << desiredLinear_ << ',' << desiredAngular_ << ','
                      << currentLinear_ << ',' << pwm << '\n';
        }

        // Publish diagnostics
        diagnostic_msgs::DiagnosticArray diagArray;
        diagArray.header.stamp = ros::Time::now();
        diagnostic_msgs::DiagnosticStatus status;
        status.name = "MotionNode";
        status.hardware_id = "motors_servo";
        if (emergencyStop_) {
            status.level = diagnostic_msgs::DiagnosticStatus::WARN;
            status.message = "Emergency stop engaged";
        } else {
            status.level = diagnostic_msgs::DiagnosticStatus::OK;
            status.message = "Operating normally";
        }
        diagArray.status.push_back(status);
        diagPub_.publish(diagArray);

        rate.sleep();
    }
    if (cmdCsv_.is_open()) cmdCsv_.close();
    if (stateCsv_.is_open()) stateCsv_.close();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_node");
    ros::NodeHandle nh;
    MotionNode node(nh);
    if (!node.initialize()) {
        ROS_FATAL("MotionNode failed to initialize. Shutting down.");
        return 1;
    }
    node.spin();
    return 0;
}
