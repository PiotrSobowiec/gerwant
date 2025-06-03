// autonomous_node.cpp
#include "autonomus/autonomous.hpp"
#include <face_marker_tracker/msg/vision_result.hpp>

AutonomousNode::AutonomousNode(const rclcpp::NodeOptions & options)
: Node("autonomous_node", options),
  has_vision_(false),
  has_slam_pose_(false),
  front_dist_(std::numeric_limits<float>::infinity()),
  fl_dist_(std::numeric_limits<float>::infinity()),
  fr_dist_(std::numeric_limits<float>::infinity()),
  rear_dist_(std::numeric_limits<float>::infinity()),
  state_(SEARCH_FACE)
{
  // 1) deklaracja parametrów statycznych
  this->declare_parameter<std::string>("vision_topic", "/vision/result");
  this->declare_parameter<std::string>("sensor_front_topic", "/sensors/front");
  this->declare_parameter<std::string>("sensor_fl_topic", "/sensors/front_left");
  this->declare_parameter<std::string>("sensor_fr_topic", "/sensors/front_right");
  this->declare_parameter<std::string>("sensor_rear_topic", "/sensors/rear");
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<std::string>("slam_pose_topic", "/orb_slam2/pose");
  this->declare_parameter<int>("frame_width", 640);

  this->declare_parameter<double>("follow_distance", 50.0);
  this->declare_parameter<double>("safe_distance", 30.0);
  this->declare_parameter<double>("max_linear_speed", 0.5);
  this->declare_parameter<double>("max_angular_speed", 1.0);

  this->get_parameter("vision_topic", vision_topic_);
  this->get_parameter("sensor_front_topic", sensor_front_topic_);
  this->get_parameter("sensor_fl_topic", sensor_fl_topic_);
  this->get_parameter("sensor_fr_topic", sensor_fr_topic_);
  this->get_parameter("sensor_rear_topic", sensor_rear_topic_);
  this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
  this->get_parameter("slam_pose_topic", slam_pose_topic_);
  this->get_parameter("frame_width", frame_width_);

  this->get_parameter("follow_distance", follow_distance_);
  this->get_parameter("safe_distance", safe_distance_);
  this->get_parameter("max_linear_speed", max_linear_);
  this->get_parameter("max_angular_speed", max_angular_);

  // 2) callback na zmianę parametrów
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&AutonomousNode::on_param_change, this, std::placeholders::_1)
  );

  // 3) subskrypcje
  vision_sub_ = this->create_subscription<face_marker_tracker::msg::VisionResult>(
    vision_topic_, 10, std::bind(&AutonomousNode::vision_callback, this, _1)
  );
  front_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    sensor_front_topic_, 10, std::bind(&AutonomousNode::front_callback, this, _1)
  );
  fl_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    sensor_fl_topic_, 10, std::bind(&AutonomousNode::fl_callback, this, _1)
  );
  fr_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    sensor_fr_topic_, 10, std::bind(&AutonomousNode::fr_callback, this, _1)
  );
  rear_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    sensor_rear_topic_, 10, std::bind(&AutonomousNode::rear_callback, this, _1)
  );
  slam_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    slam_pose_topic_, 10, std::bind(&AutonomousNode::slam_callback, this, _1)
  );

  // 4) publisher cmd_vel
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  // 5) katalog na logi
  fs::path logDir = fs::current_path() / "logs" / "autonomous";
  fs::create_directories(logDir);
  state_log_.open((logDir / "state_log.csv").string());
  slam_log_.open((logDir / "slam_log.csv").string());
  if (state_log_) state_log_ << "timestamp,state\n";
  if (slam_log_)  slam_log_  << "timestamp,px,py,pz,qx,qy,qz,qw\n";

  RCLCPP_INFO(this->get_logger(), "[AutonomousNode] initialized (frame_width=%d)", frame_width_);

  // 6) timer główny 20 Hz
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&AutonomousNode::spin, this)
  );
}

bool AutonomousNode::initialize() {
  // (wszystko już zrobione w konstruktorze)
  return true;
}

void AutonomousNode::spin() {
  State new_state = state_;

  if (front_dist_ < safe_distance_) {
    new_state = AVOID_OBSTACLE;
  } else if (state_ == AVOID_OBSTACLE && front_dist_ >= safe_distance_) {
    new_state = SEARCH_FACE;
  } else if (state_ == SEARCH_FACE && has_vision_) {
    new_state = FOLLOW_FACE;
  } else if (state_ == FOLLOW_FACE && has_vision_ && last_vision_.status == "Tracking Marker") {
    new_state = FOLLOW_MARKER;
  } else if (state_ == FOLLOW_MARKER && (!has_vision_ || last_vision_.status != "Tracking Marker")) {
    new_state = SEARCH_MARKER;
  } else if (state_ == SEARCH_MARKER && has_vision_ && last_vision_.status == "Tracking Marker") {
    new_state = FOLLOW_MARKER;
  }

  if (new_state != state_) {
    double t = this->now().seconds();
    state_log_ << t << ',' << state_to_string(new_state) << "\n";
    state_ = new_state;
  }

  geometry_msgs::msg::Twist cmd;
  switch (state_) {
    case AVOID_OBSTACLE:
      cmd.linear.x = 0.0;
      cmd.angular.z = (fl_dist_ > fr_dist_) ? max_angular_ : -max_angular_;
      break;

    case SEARCH_FACE:
      cmd.linear.x = 0.0;
      cmd.angular.z = max_angular_ / 2.0;
      break;

    case FOLLOW_FACE: {
      double cx = last_vision_.face_x + last_vision_.face_width * 0.5;
      double errX = cx - frame_width_ * 0.5;
      cmd.angular.z = -(errX / (frame_width_ * 0.5)) * max_angular_;
      double errD = follow_distance_ / std::max(0.1f, front_dist_);
      cmd.linear.x = std::clamp(errD, 0.0, max_linear_);
      break;
    }

    case SEARCH_MARKER:
      cmd.linear.x = 0.0;
      cmd.angular.z = max_angular_ / 2.0;
      break;

    case FOLLOW_MARKER: {
      double errX = last_vision_.marker_center_x - frame_width_ * 0.5;
      cmd.angular.z = -(errX / (frame_width_ * 0.5)) * max_angular_;
      double errD = front_dist_ - follow_distance_;
      cmd.linear.x = std::clamp(errD / follow_distance_, 0.0, max_linear_);
      break;
    }
  }

  cmd_pub_->publish(cmd);
}

rcl_interfaces::msg::SetParametersResult AutonomousNode::on_param_change(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  for (const auto &p : params) {
    if (p.get_name() == "follow_distance") {
      follow_distance_ = p.as_double();
    } else if (p.get_name() == "safe_distance") {
      safe_distance_ = p.as_double();
    } else if (p.get_name() == "max_linear_speed") {
      max_linear_ = p.as_double();
    } else if (p.get_name() == "max_angular_speed") {
      max_angular_ = p.as_double();
    }
  }

  RCLCPP_INFO(this->get_logger(),
    "[Params changed] follow=%.1f safe=%.1f lin=%.2f ang=%.2f",
    follow_distance_, safe_distance_, max_linear_, max_angular_);

  return result;
}

void AutonomousNode::vision_callback(
  const face_marker_tracker::msg::VisionResult::SharedPtr msg)
{
  last_vision_ = *msg;
  has_vision_  = true;
}

void AutonomousNode::front_callback(const std_msgs::msg::Float32::SharedPtr msg) {
  front_dist_ = msg->data;
}
void AutonomousNode::fl_callback(const std_msgs::msg::Float32::SharedPtr msg) {
  fl_dist_ = msg->data;
}
void AutonomousNode::fr_callback(const std_msgs::msg::Float32::SharedPtr msg) {
  fr_dist_ = msg->data;
}
void AutonomousNode::rear_callback(const std_msgs::msg::Float32::SharedPtr msg) {
  rear_dist_ = msg->data;
}

void AutonomousNode::slam_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  has_slam_pose_ = true;
  last_pose_     = msg->pose;
  double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  slam_log_ << t << ','
            << last_pose_.position.x << ','
            << last_pose_.position.y << ','
            << last_pose_.position.z << ','
            << last_pose_.orientation.x << ','
            << last_pose_.orientation.y << ','
            << last_pose_.orientation.z << ','
            << last_pose_.orientation.w << '\n';
}

std::string AutonomousNode::state_to_string(State s) const {
  switch (s) {
    case SEARCH_FACE:   return "SEARCH_FACE";
    case FOLLOW_FACE:   return "FOLLOW_FACE";
    case SEARCH_MARKER: return "SEARCH_MARKER";
    case FOLLOW_MARKER: return "FOLLOW_MARKER";
    case AVOID_OBSTACLE:return "AVOID_OBSTACLE";
  }
  return "UNKNOWN";
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutonomousNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
