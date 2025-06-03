// motion_node.cpp
#include "motion_node/motion.hpp"

MotionNode::MotionNode(const rclcpp::NodeOptions & options)
: Node("motion_node", options)
{
  // W konstruktorze nie wczytujemy parametrów ani hardware’u—
  // robimy to w initialize(), żeby móc zwracać false, jeśli coś się nie uda.
}

bool MotionNode::initialize() {
  loadParameters();

  // 1) subskrypcja /cmd_vel
  cmdVelSub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmdVelTopic_, 10,
    std::bind(&MotionNode::cmdVelCallback, this, std::placeholders::_1)
  );

  // 2) publisher diagnostyk
  diagPub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/motion/diagnostics", 1
  );

  // 3) serwis do awaryjnego zatrzymania
  emergencySrv_ = this->create_service<std_srvs::srv::SetBool>(
    "emergency_stop",
    std::bind(&MotionNode::emergencyStopCallback, this,
              std::placeholders::_1, std::placeholders::_2)
  );

  // 4) przygotuj katalog na logi
  std::filesystem::path logDir = std::filesystem::current_path() / "logs" / "motion";
  std::filesystem::create_directories(logDir);

  std::string cmdFile   = (logDir / "commands.csv").string();
  std::string stateFile = (logDir / "state.csv").string();
  cmdCsv_.open(cmdFile);
  stateCsv_.open(stateFile);
  if (cmdCsv_)   cmdCsv_ << "timestamp,linear_x,angular_z\n";
  if (stateCsv_) stateCsv_ << "timestamp,desired_linear,desired_angular,current_linear,pwm\n";

  // 5) inicjalizuj stan
  lastCmdTime_  = this->now().seconds();
  emergencyStop_ = false;
  desiredLinear_ = 0.0;
  desiredAngular_ = 0.0;
  currentLinear_ = 0.0;

  // 6) timer sterowania w zadanej częstotliwości controlRate_
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(int(1000.0 / controlRate_)),
    std::bind(&MotionNode::spin, this)
  );

  RCLCPP_INFO(this->get_logger(),
              "MotionNode initialized, listening on '%s'", cmdVelTopic_.c_str());
  return true;
}

void MotionNode::spin() {
  // Ta metoda będzie wywoływana przez timer, np. 10 Hz lub 20 Hz
  double now = this->now().seconds();
  if ((now - lastCmdTime_) > cmdTimeout_) {
    desiredLinear_ = 0.0;
    desiredAngular_ = 0.0;
  }

  double delta = maxAccel_ / controlRate_;
  if (currentLinear_ < desiredLinear_) {
    currentLinear_ = std::min(currentLinear_ + delta, desiredLinear_);
  } else {
    currentLinear_ = std::max(currentLinear_ - delta, desiredLinear_);
  }

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
    if (desiredAngular_ > 0.1)       servo_.turn(LEFT);
    else if (desiredAngular_ < -0.1) servo_.turn(RIGHT);
    else                              servo_.turn(STRAIGHT);
  }

  if (stateCsv_) {
    stateCsv_ << now << ',' << desiredLinear_ << ',' << desiredAngular_
              << ',' << currentLinear_ << ',' << pwm << '\n';
  }

  // publish diagnostics
  diagnostic_msgs::msg::DiagnosticArray diagArray;
  diagArray.header.stamp = this->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "MotionNode";
  status.hardware_id = "motors_servo";
  if (emergencyStop_) {
    status.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Emergency stop engaged";
  } else {
    status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Operating normally";
  }
  diagArray.status.push_back(status);
  diagPub_->publish(diagArray);
}

void MotionNode::loadParameters() {
  // w ROS 2 wczytujemy parametry inaczej niż w ROS 1
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<double>("control_rate", 10.0);
  this->declare_parameter<double>("cmd_timeout", 1.0);
  this->declare_parameter<double>("max_speed", 1.0);
  this->declare_parameter<double>("max_accel", 0.5);
  this->declare_parameter<std::vector<int>>("wheel_pins", {2,3,4,5,6,7,8,9,10,11,12,13});
  this->declare_parameter<int>("servo_pin", 14);

  this->get_parameter("cmd_vel_topic", cmdVelTopic_);
  this->get_parameter("control_rate", controlRate_);
  this->get_parameter("cmd_timeout", cmdTimeout_);
  this->get_parameter("max_speed", maxSpeed_);
  this->get_parameter("max_accel", maxAccel_);

  std::vector<int> wheelPins;
  this->get_parameter("wheel_pins", wheelPins);
  int servoPin; this->get_parameter("servo_pin", servoPin);

  motors_.clear();
  if (wheelPins.size() == 12) {
    for (int i = 0; i < 12; i += 3) {
      motors_.emplace_back(wheelPins[i], wheelPins[i + 1], wheelPins[i + 2]);
    }
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Expected 12 wheel pins, got %zu", wheelPins.size());
  }
  servo_ = ServoController(servoPin);
}

void MotionNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  double now = this->now().seconds();
  lastCmdTime_ = now;
  desiredLinear_  = std::clamp(msg->linear.x, -maxSpeed_, maxSpeed_);
  desiredAngular_ = msg->angular.z;

  if (cmdCsv_) {
    cmdCsv_ << now << ',' << msg->linear.x << ',' << msg->angular.z << '\n';
  }
}

void MotionNode::emergencyStopCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  emergencyStop_ = request->data;
  if (emergencyStop_) {
    stopMotors();
    response->message = "Emergency stop engaged";
    response->success = true;
  } else {
    response->message = "Emergency stop released";
    response->success = true;
  }
}

void MotionNode::stopMotors() {
  for (auto &motor : motors_) {
    motor.setDirection(STOP);
    motor.setSpeed(0);
  }
  servo_.turn(STRAIGHT);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionNode>();
  if (!node->initialize()) {
    RCLCPP_FATAL(node->get_logger(),
                 "MotionNode failed to initialize. Shutting down.");
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
