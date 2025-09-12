#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <gripper_motor/GripperMotor.h>

#include <map>
#include <vector>
#include <string>
#include <limits>
#include <cmath>
#include <algorithm>

namespace {

// ticks→rad with configurable resolution and gear/sign/offset
inline double ticks_to_rad(int16_t ticks,
                           double resolution_ticks,
                           double gear_ratio,
                           double sign,
                           double offset_rad) {
  // motor_electrical_angle = ticks/resolution * 2π
  const double rad = (static_cast<double>(ticks) / resolution_ticks) * 2.0 * M_PI;
  return sign * rad * gear_ratio + offset_rad;
}

inline double ticks_per_s_to_rad_per_s(int16_t v_ticks_per_s,
                                       double resolution_ticks,
                                       double gear_ratio,
                                       double sign) {
  const double radps = (static_cast<double>(v_ticks_per_s) / resolution_ticks) * 2.0 * M_PI;
  return sign * radps * gear_ratio;
}

} // namespace

class GripperJointStateNode : public rclcpp::Node {
public:
  GripperJointStateNode()
  : rclcpp::Node("gripper_joint_state_node")
  {
    // ---- Params ----
    serial_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB1");
    auto ids_param       = this->declare_parameter<std::vector<int64_t>>("ids", {1});
    publish_rate_hz_     = this->declare_parameter<double>("publish_rate_hz", 20.0);
    topic_               = this->declare_parameter<std::string>("topic", "berry_gripper/joint_state");

    names_ = this->declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{});

    // Kinematic params (per joint; scalar expands if needed)
    resolution_ticks_ = this->declare_parameter<double>("resolution_ticks", 4096.0); // scalar default
    gear_ratio_vec_   = this->declare_parameter<std::vector<double>>("gear_ratio",   std::vector<double>{1.0});
    sign_vec_         = this->declare_parameter<std::vector<double>>("sign",         std::vector<double>{+1.0});
    offset_vec_       = this->declare_parameter<std::vector<double>>("offset_rad",   std::vector<double>{0.0});

    // Convert IDs to uint8_t
    ids_.reserve(ids_param.size());
    for (auto v : ids_param) ids_.push_back(static_cast<uint8_t>(v));

    // Default names if empty
    if (names_.empty()) {
      names_.reserve(ids_.size());
      for (auto id : ids_) names_.push_back("joint_" + std::to_string(id));
    }

    // Expand scalar vectors to match joint count
    const size_t N = ids_.size();
    auto expand = [N](std::vector<double>& v, double fill) {
      if (v.empty()) v.push_back(fill);
      if (v.size() == 1 && N > 1) v.resize(N, v.front());
    };
    expand(gear_ratio_vec_, 1.0);
    expand(sign_vec_,       +1.0);
    expand(offset_vec_,     0.0);

    // Validate sizes
    if (names_.size() != N) {
      RCLCPP_FATAL(get_logger(), "joint_names size (%zu) must match ids size (%zu)", names_.size(), N);
      throw std::runtime_error("joint_names size mismatch");
    }
    if (gear_ratio_vec_.size() != N || sign_vec_.size() != N || offset_vec_.size() != N) {
      RCLCPP_FATAL(get_logger(),
        "gear_ratio/sign/offset sizes must all equal ids size (%zu). Got: %zu/%zu/%zu",
        N, gear_ratio_vec_.size(), sign_vec_.size(), offset_vec_.size());
      throw std::runtime_error("kinematic vector size mismatch");
    }

    // ---- Open serial/motor ----
    if (!motor_.open(serial_)) {
      RCLCPP_FATAL(get_logger(), "Failed to open serial: %s", serial_.c_str());
      throw std::runtime_error("serial open failed");
    }
    RCLCPP_INFO(get_logger(), "Opened %s", serial_.c_str());

    // ---- Publisher & prealloc message ----
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(topic_, rclcpp::SensorDataQoS());
    msg_.name = names_;
    msg_.position.assign(N, std::numeric_limits<double>::quiet_NaN());
    msg_.velocity.assign(N, std::numeric_limits<double>::quiet_NaN());
    // msg_.effort left empty

    // ---- Timer ----
    using namespace std::chrono;
    const auto period = duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = this->create_wall_timer(
      duration_cast<milliseconds>(period),
      std::bind(&GripperJointStateNode::on_timer, this)
    );
  }

  ~GripperJointStateNode() override {
    motor_.close();
  }

private:
  void on_timer() {
    std::map<uint8_t, berrygripper::Feedback> out;
    if (!motor_.readPosSpeed(ids_, out)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "readPosSpeed failed");
      return;
    }

    msg_.header.stamp = this->now();
    const double res = resolution_ticks_;

    for (size_t i = 0; i < ids_.size(); ++i) {
      const uint8_t id = ids_[i];
      const auto it = out.find(id);
      if (it == out.end()) {
        msg_.position[i] = std::numeric_limits<double>::quiet_NaN();
        msg_.velocity[i] = std::numeric_limits<double>::quiet_NaN();
        continue;
      }
      const auto &fb = it->second;
      const double gear   = gear_ratio_vec_[i];
      const double sign   = sign_vec_[i];
      const double offset = offset_vec_[i];

      msg_.position[i] = ticks_to_rad(fb.position, res, gear, sign, offset);
      msg_.velocity[i] = fb.speed;
    }

    pub_->publish(msg_);
  }

private:
  // Params
  std::string serial_;
  std::string topic_;
  double publish_rate_hz_{20.0};
  double resolution_ticks_{4096.0};
  std::vector<uint8_t> ids_;
  std::vector<std::string> names_;
  std::vector<double> gear_ratio_vec_, sign_vec_, offset_vec_;

  // ROS
  berrygripper::GripperMotor motor_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState msg_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GripperJointStateNode>());
  } catch (const std::exception& e) {
    std::cerr << "Fatal: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
