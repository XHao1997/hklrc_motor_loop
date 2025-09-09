#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <gripper_motor/GripperMotor.h>

#include <map>
#include <vector>
#include <string>
#include <limits>
#include <cmath>

namespace {

inline double ticks_to_rad(int16_t ticks) {
  // 0..4095 ticks -> 0..2π rad
  return (static_cast<double>(ticks) / 4096.0) * 2.0 * M_PI;
}
inline double ticks_per_s_to_rad_per_s(int16_t v_ticks_per_s) {
  return (static_cast<double>(v_ticks_per_s) / 4096.0) * 2.0 * M_PI;
}

} // namespace

class GripperJointStateNode : public rclcpp::Node {
public:
  GripperJointStateNode()
  : rclcpp::Node("gripper_joint_state_node")
  {
    // Params
    serial_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    auto ids_param = this->declare_parameter<std::vector<int64_t>>("ids", {1});
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 25.0);
    names_ = this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});

    // IDs -> uint8_t
    ids_.reserve(ids_param.size());
    for (auto v : ids_param) ids_.push_back(static_cast<uint8_t>(v));

    // Default names if empty
    if (names_.empty()) {
      names_.reserve(ids_.size());
      for (auto id : ids_) names_.push_back("joint_" + std::to_string(id));
    }
    if (names_.size() != ids_.size()) {
      RCLCPP_FATAL(get_logger(), "joint_names size (%zu) must match ids size (%zu)", names_.size(), ids_.size());
      throw std::runtime_error("joint_names size mismatch");
    }

    // Open serial
    if (!motor_.open(serial_)) {
      RCLCPP_FATAL(get_logger(), "Failed to open serial: %s", serial_.c_str());
      throw std::runtime_error("serial open failed");
    }
    RCLCPP_INFO(get_logger(), "Opened %s", serial_.c_str());

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("berry_gripper/joint_state", rclcpp::QoS(10));

    // Timer
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

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = names_;
    msg.position.resize(names_.size());
    msg.velocity.resize(names_.size());
    // (effort left empty)

    for (size_t i = 0; i < ids_.size(); ++i) {
      uint8_t id = ids_[i];
      auto it = out.find(id);
      if (it == out.end()) {
        msg.position[i] = std::numeric_limits<double>::quiet_NaN();
        msg.velocity[i] = std::numeric_limits<double>::quiet_NaN();
        continue;
      }
      const auto &fb = it->second;
      msg.position[i] = ticks_to_rad(fb.position);
      msg.velocity[i] = ticks_per_s_to_rad_per_s(fb.speed);
    }

    pub_->publish(std::move(msg));
  }

private:
  std::string serial_;
  double publish_rate_hz_{20.0};
  std::vector<uint8_t> ids_;
  std::vector<std::string> names_;

  berrygripper::GripperMotor motor_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<GripperJointStateNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::cerr << "Fatal: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
