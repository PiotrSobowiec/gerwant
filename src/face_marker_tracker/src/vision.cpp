#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/face.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/aruco.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <algorithm>

#include "face_marker_tracker/vision.hpp"

using namespace cv;
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

VisionSystem::VisionSystem()
  : frame_counter_(0), last_fps_(0.0), test_id_(1), face_verified(false), is_tracking(false)
{
    dictionary = makePtr<aruco::Dictionary>(
        aruco::getPredefinedDictionary(aruco::DICT_4X4_50)
    );
}

bool VisionSystem::initialize(int model_choice, int tracker_choice) {
    model_choice_ = model_choice;
    tracker_choice_ = tracler_choice;
    // Prepare model/tracker names
    modelName_   = (model_choice_ == 1 ? "Haar" : model_choice_ == 2 ? "SSD" : "LBP");
    trackerName_ = (tracker_choice_ == 1 ? "CSRT" : tracker_choice_ == 2 ? "KCF" : "MOSSE");

    // Create logs directory
    fs::create_directories("logs/vision");
    std::string logPath = "logs/vision/" + modelName_ + "_" + trackerName_
                        + "_test" + std::to_string(test_id_) + ".csv";
    log_file_.open(logPath);
    if (!log_file_) {
        RCLCPP_ERROR(node_->get_logger(), "Nie można otworzyć pliku log: %s", logPath.c_str());
        return false;
    }
    // CSV header
    log_file_ << "TestID,Frame,FPS,Mode,Person,Confidence,Model,Tracker,FaceTime[ms],MarkerTime[ms]\n";

    // Initialize timing
    last_time_ = Clock::now();

    // Paths
    std::string face_data_path      = "/home/piotrek/catkin_ws/src/face_marker_tracker/faces"; // DO ZMIANY ŚCIEŻKI BO SĄ INNE TERAZ
    std::string face_cascade_path  = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml";

    // Load face detector
    if (model_choice_ == 1) { // Haar
        if (!face_cascade_.load(face_cascade_path)) {
            RCLCPP_ERROR(node_->get_logger(), "Nie można załadować Haar Cascade!");
            return false;
        }
    } else if (model_choice_ == 2) { // SSD
        ssd_net_ = dnn::readNetFromCaffe(
            "../classifiers/deploy.prototxt",
            "../classifiers/res10_300x300_ssd_iter_140000.caffemodel"
        );
        if (ssd_net.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "Nie można załadować SSD!");
            return false;
        }
    } else { // LBP
        if (!face_cascade_.load("../classifiers/lbpcascade_frontalface.xml")) {
            RCLCPP_ERROR(node_->get_logger(), "Nie można załadować LBP Cascade!");
            return false;
        }
    }

    // Train LBPH model
    face_model = cv::face::LBPHFaceRecognizer::create();
    std::vector<Mat> images;
    std::vector<int> labels;
    int label_counter = 0;
    for (auto &person : fs::directory_iterator(face_data_path)) {
        if (!fs::is_directory(person)) continue;
        std::string name = person.path().filename().string();
        int label = label_counter++;
        label_names_[label] = name;
        for (auto &img_path : fs::directory_iterator(person.path())) {
            std::string ext = img_path.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            if (ext != ".jpg" && ext != ".jpeg" && ext != ".png") continue;
            Mat img = imread(img_path.path().string(), IMREAD_GRAYSCALE);
            if (!img.empty()) {
                resize(img, img, Size(100, 100));
                images.push_back(img);
                labels.push_back(label);
            }
        }
    }
    if (!images.empty()) {
        face_model_->train(images, labels);
        } else {
            RCLCPP_WARN(node_->get_logger(), "Brak zdjęć do treningu LBPH!");
        }

    return true;
}

VisionResult VisionSystem::process_frame(const Mat& frame) {
    VisionResult result;
    double face_time_ms = 0.0, marker_time_ms = 0.0;

    // Compute FPS
    auto now = Clock::now();
    if (frame_counter_ > 0) {
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time_).count();
        last_fps_ = diff > 0 ? 1000.0 / diff : 0.0;
    }
    last_time_ = now;
    frame_counter_++;

    // Preprocess
    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    result.face_verified    = face_verified;
    result.marker_tracking  = is_tracking;
    result.recognized_label = confirmed_label;
    result.recognized_name  = recognized_name;
    result.confidence       = last_confidence;

    // 1) Face detection/recognition
    if (!face_verified) {
        std::vector<Rect> faces;
        auto t1 = Clock::now();
        if (model_choice_ == 2) {
            Mat blob = dnn::blobFromImage(frame, 1.0, Size(300,300), Scalar(104,177,123), false, false);
            ssd_net_.setInput(blob);
            Mat det = ssd_net_.forward();
            Mat detMat(det.size[2], det.size[3], CV_32F, det.ptr<float>());
            for (int i = 0; i < detMat.rows; i++) {
                float conf = detMat.at<float>(i,2);
                if (conf > 0.5f) {
                    int x1 = int(detMat.at<float>(i,3)*frame.cols);
                    int y1 = int(detMat.at<float>(i,4)*frame.rows);
                    int x2 = int(detMat.at<float>(i,5)*frame.cols);
                    int y2 = int(detMat.at<float>(i,6)*frame.rows);
                    faces.push_back(Rect(Point(x1,y1), Point(x2,y2)));
                }
            }
        } else {
            face_cascade_.detectMultiScale(gray, faces, 1.2, 5);
        }
        auto t2 = Clock::now();
        face_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

        for (auto &face : faces) {
            Mat roi = gray(face);
            resize(roi, roi, Size(100,100));
            int label; double conf;
            face_model_->predict(roi, label, conf);
            last_confidence = conf;
            if (conf < 200.0) {
                face_verified   = true;
                confirmed_label = label;
                recognized_name = label_names_[label];
                show_welcome    = true;
                result.face_verified    = true;
                result.recognized_label = label;
                result.recognized_name  = recognized_name;
                result.confidence       = static_cast<float>(conf);
                break;
            }
        }
        result.status= "Tryb: Wyszukiwanie twarzy";
        // Log and return
        if (log_file_) {
            log_file_ << test_id_<<','<<frame_counter_<<','<<last_fps_<<','
            << result.status << ',' << result.recognized_name << ',' << result.confidence << ','
            << modelName_ << ',' << trackerName_ << ',' << face_time_ms << ',' << marker_time_ms << '\n';
        }
        return result;
    }

    // 2) Marker detection/init tracker
    if (face_verified && !is_tracking) {
        auto t3 = Clock::now();
        std::vector<int> ids;
        std::vector<std::vector<Point2f>> corners;
        aruco::detectMarkers(frame, dictionary, corners, ids);
        for (size_t i = 0; i < ids.size(); i++) {
            if (ids[i] == AUTHORIZED_MARKER_ID) {
                Rect box = boundingRect(corners[i]);
                if      (tracker_choice==1) tracker_ = TrackerCSRT::create();
                else if (tracker_choice==2) tracker_ = TrackerKCF::create();
                else                         tracker_ = TrackerMOSSE::create();
                tracker_->init(frame, box);
                target_box = box;
                is_tracking = true;
                result.marker_tracking = true;
                result.marker_box      = box;
                result.status_text     = "Tryb: Sledzenie markera";
                auto t4 = Clock::now();
                marker_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
                // Log and return
                if (log_file_){
                    log_file_ << test_id_<<','<<frame_counter_<<','<<last_fps_<<','
                    << result.status << ',' << result.recognized_name << ',' << result.confidence << ','
                    << modelName_ << ',' << trackerName_ << ',' << face_time_ms << ',' << marker_time_ms << '\n';
                }
                return result;
            }
        }
        // timeout logic
        result.status = "Tryb: Twarz rozpoznana - szukam markera";
        if (log_file_){ 
            log_file_ << test_id_<<','<<frame_counter_<<','<<last_fps_<<','
            << result.status << ',' << result.recognized_name << ',' << result.confidence << ','
            << modelName_ << ',' << trackerName_ << ',' << face_time_ms << ',' << marker_time_ms << '\n';
        }
        return result;
    }

    // 3) Tracking
    if (is_tracking && tracker_) {
        auto t5 = Clock::now();
        bool ok = tracker_->update(frame, target_box);
        auto t6 = Clock::now();
        marker_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t5).count();
        if (ok) {
            result.marker_tracking = true;
            result.marker_box      = target_box;
            result.status     = "Tryb: Sledzenie markera";
        } else {
            is_tracking = false;
            face_verified = false;
            confirmed_label = -1;
            recognized_name.clear();
            result.marker_tracking = false;
            result.face_verified   = false;
            result.status = "Tryb: Utrata sledzenia";
        }
        if (log_file_){
            log_file_ << test_id_<<','<<frame_counter_<<','<<last_fps_<<','
            << result.status << ',' << result.recognized_name << ',' << result.confidence << ','
            << modelName_ << ',' << trackerName_ << ',' << face_time_ms << ',' << marker_time_ms << '\n';
        }
        return result;
    }

    // Default
    result.status_text = "Tryb: Nieaktywny";
    if (log_file_) {
        log_file_ << test_id_<<','<<frame_counter_<<','<<last_fps_<<','
        << result.status << ',' << result.recognized_name << ',' << result.confidence << ','
        << modelName_ << ',' << trackerName_ << ',' << face_time_ms << ',' << marker_time_ms << '\n';
    }
    return result;
}

void VisionSystem::draw_status(Mat &frame, const VisionResult &result) {
    putText(frame, result.status, Point(10,30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,255), 2);
    if (result.face_verified)
        putText(frame, "Rozpoznano: " + result.recognized_name, Point(10,60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,200,255), 2);
    if (result.marker_tracking)
        rectangle(frame, result.marker_box, Scalar(255,0,0), 2);
}
