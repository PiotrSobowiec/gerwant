// vision.hpp
#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>

// ROS parameters (private namespace, e.g., ~camera_topic)
// camera_topic: std::string (domyślnie "/camera/image_raw")
// detector_type: int (0=HAAR_CASCADE,1=LBP_CASCADE,2=SSD_MOBILENET)
// tracker_type: int (0=CSRT_TRACKER,1=KCF_TRACKER,2=MOSSE_TRACKER)
// detection_threshold: double (0.0-1.0, domyślnie 0.5)

// Typy detektorów twarzy
enum FaceDetectorType {
    HAAR_CASCADE = 0,
    LBP_CASCADE = 1,
    SSD_MOBILENET = 2
};

// Typy algorytmów śledzenia
enum TrackerType {
    CSRT_TRACKER = 0,
    KCF_TRACKER = 1,
    MOSSE_TRACKER = 2
};

// Struktura wyników detekcji/śledzenia
typedef struct {
    int frameNumber;
    float fps;
    std::string status;      // "Detecting", "Tracking", "MarkerFound"
    std::string personName;
    float confidence;
    cv::Rect faceRect;
    cv::Point2f markerCenter;
} VisionResult;

// Klasa modułu wizyjnego obsługująca detekcję i śledzenie
typedef void (*ResultCallback)(const VisionResult&);
class VisionSystem {
public:
    VisionSystem(ros::NodeHandle& nh);
    bool initialize(); // wczytuje parametry i subskrybuje obraz
    void registerCallback(ResultCallback cb);
    void spinOnce();   // przetwarza ostatnią klatkę
    void shutdown();

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void loadParameters();

    ros::NodeHandle nh_;
    ros::Subscriber imageSub_;
    FaceDetectorType detectorType_;
    TrackerType trackerType_;
    double detectionThreshold_;
    std::string cameraTopic_;

    cv::CascadeClassifier haarCascade_;
    cv::CascadeClassifier lbpCascade_;
    cv::dnn::Net ssdNet_;
    cv::Ptr<cv::Tracker> tracker_;
    bool isTracking_;
    cv::Mat lastFrame_;

    ResultCallback callback_;
};

