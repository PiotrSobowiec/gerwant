// vision_node.cpp
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <vision/vision.hpp>
#include <vision_msgs/VisionResult.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;

    // Initialize VisionSystem
    vision::VisionSystem vsys(nh);
    if (!vsys.initialize()) {
        ROS_FATAL("Failed to initialize VisionSystem");
        return 1;
    }

    // Publisher for VisionResult
    ros::Publisher resultPub = nh.advertise<vision_msgs::VisionResult>("/vision/result", 10);

    // Register callback to convert and publish results
    vsys.registerCallback([&](const vision::VisionResult& res) {
        vision_msgs::VisionResult msg;
        msg.header.stamp = ros::Time::now();
        msg.frame_number = res.frameNumber;
        msg.fps = res.fps;
        msg.status = res.status;
        msg.person_name = res.personName;
        msg.confidence = res.confidence;
        msg.face_rect.x = res.faceRect.x;
        msg.face_rect.y = res.faceRect.y;
        msg.face_rect.width = res.faceRect.width;
        msg.face_rect.height = res.faceRect.height;
        msg.marker_center.x = res.markerCenter.x;
        msg.marker_center.y = res.markerCenter.y;
        resultPub.publish(msg);
    });

    ros::Rate rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        vsys.spinOnce();
        rate.sleep();
    }
    vsys.shutdown();
    return 0;
}
