// vision_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include "face_marker_tracker/vision.hpp"
#include "face_marker_tracker/msg/vision_result.hpp"

using std::placeholders::_1;
using VisionResultMsg = face_marker_tracker::msg::VisionResult;

class VisionNode : public rclcpp::Node {
public:
  VisionNode()
  : Node("vision_node")
  {
    // 1) Stwórz instancję VisionSystem
    this->declare_parameter<std::string>("camera_topic", "/camera/image_raw");
    this->declare_parameter<int>("detector_type", 1);   // 1 = Haar
    this->declare_parameter<int>("tracker_type", 1);    // 1 = CSRT

    this->get_parameter("camera_topic", camera_topic_);
    int det = this->get_parameter("detector_type").as_int();
    int trk = this->get_parameter("tracker_type").as_int();

    vsys_ = std::make_shared<VisionSystem>(shared_from_this());
    if (!vsys_->initialize(det, trk)) {
      RCLCPP_FATAL(this->get_logger(), "VisionSystem initialization failed");
      rclcpp::shutdown();
      return;
    }

    img_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    img_sub_ = img_transport_->subscribe(
      camera_topic_, 10,
      std::bind(&VisionNode::image_callback, this, _1)
    );

    // 2) Publisher wyników
    resultPub_ = this->create_publisher<VisionResultMsg>("/vision/result", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&VisionNode::timer_callback, this)
    );
  }

    // 3) Rejestracja callbacku, który konwertuje VisionResult → ROS2‐owy msg
    vsys_->registerCallback(
      [this](const VisionResult& res) {
        auto msg = VisionResultMsg();
        msg.header.stamp = this->now();
        msg.frame_number = res.frameNumber;
        msg.fps          = res.fps;
        msg.status       = res.status;
        msg.person_name  = res.personName;
        msg.confidence   = res.confidence;
        msg.face_rect.x      = res.faceRect.x;
        msg.face_rect.y      = res.faceRect.y;
        msg.face_rect.z      = res.faceRect.width;
        msg.marker_center.x  = res.markerCenter.x;
        msg.marker_center.y  = res.markerCenter.y;
        resultPub_->publish(msg);
      }
    );

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      [this]() {
        vsys_->spinOnce();
      }
    );
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      latest_frame_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void timer_callback() {
    if (latest_frame_.empty()) return;

    VisionResult res = vsys_->process_frame(latest_frame_);

    VisionResultMsg msg;
    msg.header.stamp = this->now();
    msg.frame_number = res.frameNumber;
    msg.fps          = res.fps;
    msg.status       = res.status;
    msg.person_name  = res.personName;
    msg.confidence   = res.confidence;

    // face_rect (x,y,width,height)
    msg.face_x = static_cast<float>(res.faceRect.x);
    msg.face_y = static_cast<float>(res.faceRect.y);
    msg.face_width  = static_cast<float>(res.faceRect.width);
    msg.face_height = static_cast<float>(res.faceRect.height);

    msg.marker_center_x = res.markerCenter.x;
    msg.marker_center_y = res.markerCenter.y;

    result_pub_->publish(msg);
  }

  rclcpp::Publisher<VisionResultMsg>::SharedPtr result_pub_;
  std::shared_ptr<image_transport::ImageTransport> img_transport_;
  image_transport::Subscriber img_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::Mat latest_frame_;
  std::shared_ptr<VisionSystem> vsys_;
  std::string camera_topic_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}