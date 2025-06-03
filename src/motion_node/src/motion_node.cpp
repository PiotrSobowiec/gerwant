// motion_node.cpp
#include <ros/ros.h>
#include "motion_node/motion.hpp"

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
