// sensors_node.cpp
#include <ros/ros.h>
#include "sensors_node/sensors.hpp"

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
