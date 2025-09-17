// src/gripper_driver_node.cpp  (ROS 2 Humble compatible)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <map>
#include <vector>
#include <string>
#include <limits>
#include <cmath>
#include "gripper_motor/GripperMotor.h"
#include "hklrc_motor_loop/srv/set_gripper_position.hpp"

using namespace std::chrono_literals;
using berrygripper::GripperMotor;
using berrygripper::WriteOptions;
using berrygripper::WaitMode;

class GripperDriverNode : public rclcpp::Node {
public:
  GripperDriverNode() : Node("gripper_driver_node")
  {
    // ---- 参数 ----
    port_  = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baud_  = declare_parameter<int>("baud", 1'000'000);

    ids_param_ = declare_parameter<std::vector<int64_t>>("ids", {1});
    ids_.reserve(ids_param_.size());
    for (auto v : ids_param_) ids_.push_back(static_cast<uint8_t>(v));

    pub_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
    js_topic_    = declare_parameter<std::string>("joint_state_topic", "berry_gripper/joint_state");
    frame_id_    = declare_parameter<std::string>("frame_id", "");

    resolution_ticks_ = declare_parameter<double>("resolution_ticks", 4096.0);
    gear_   = declare_parameter<std::vector<double>>("gear_ratio",   std::vector<double>(ids_.size(), 1.0));
    sign_   = declare_parameter<std::vector<double>>("sign",         std::vector<double>(ids_.size(), +1.0));
    offset_ = declare_parameter<std::vector<double>>("offset_rad",   std::vector<double>(ids_.size(),  0.0));

    // ---- 打开串口 ----
    if (!motor_.open(port_, baud_)) {
      RCLCPP_FATAL(get_logger(), "Failed to open %s @ %d", port_.c_str(), baud_);
      throw std::runtime_error("serial open failed");
    }
    RCLCPP_INFO(get_logger(), "Opened %s @ %d", port_.c_str(), baud_);

    // ---- CallbackGroup：互斥 ---- (Humble 里依然可用)
    cg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // ---- JointState 发布器 ----
    js_pub_ = create_publisher<sensor_msgs::msg::JointState>(js_topic_, rclcpp::SensorDataQoS());
    js_msg_.name.reserve(ids_.size());
    for (auto id : ids_) js_msg_.name.push_back("joint_" + std::to_string(id));
    js_msg_.position.assign(ids_.size(), std::numeric_limits<double>::quiet_NaN());
    js_msg_.velocity.assign(ids_.size(), std::numeric_limits<double>::quiet_NaN());

    // ---- 定时器读取位置/速度并发布 ----
    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, pub_rate_hz_));
    // Humble：把 callback_group 作为 create_wall_timer 的最后一个参数传入
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&GripperDriverNode::on_timer, this),
      cg_);

    // ---- Service：设置目标位置 ----
    // Humble：create_service(… , qos_profile, callback_group)
    srv_ = this->create_service<hklrc_motor_loop::srv::SetGripperPosition>(
      "set_gripper_position",
      std::bind(&GripperDriverNode::on_set_pos, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      cg_);
  }

  ~GripperDriverNode() override {
    motor_.close();
  }

private:
  // tick->rad
  static double ticks_to_rad(int32_t ticks, double res, double gear, double sign, double offset){
    const double rad = (static_cast<double>(ticks) / res) * 2.0 * M_PI;
    return sign * rad * gear + offset;
  }
  // ticks/s -> rad/s
  static double ticksps_to_radps(int32_t v, double res, double gear, double sign){
    const double radps = (static_cast<double>(v) / res) * 2.0 * M_PI;
    return sign * radps * gear;
  }

  void on_timer(){
    std::map<uint8_t, berrygripper::Feedback> out;
    {
      // std::lock_guard<std::mutex> lk(io_mtx_);
      if (!motor_.readPosSpeed(ids_, out)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "readPosSpeed failed");
        return;
      }
    }

    js_msg_.header.stamp = now();
    if (!frame_id_.empty()) js_msg_.header.frame_id = frame_id_;

    for (size_t i = 0; i < ids_.size(); ++i) {
      auto it = out.find(ids_[i]);
      if (it == out.end()) {
        js_msg_.position[i] = std::numeric_limits<double>::quiet_NaN();
        js_msg_.velocity[i] = std::numeric_limits<double>::quiet_NaN();
        continue;
      }
      const auto &fb = it->second;
      js_msg_.position[i] = ticks_to_rad(fb.position, resolution_ticks_, gear_[i], sign_[i], offset_[i]);
      js_msg_.velocity[i] = ticksps_to_radps(fb.speed,  resolution_ticks_, gear_[i], sign_[i]);
    }
    js_pub_->publish(js_msg_);
  }

  void on_set_pos(
      const std::shared_ptr<hklrc_motor_loop::srv::SetGripperPosition::Request> req,
      std::shared_ptr<hklrc_motor_loop::srv::SetGripperPosition::Response> resp)
  {
    try {
      // std::lock_guard<std::mutex> lk(io_mtx_);
      motor_.writePos(static_cast<uint8_t>(req->id),
                      req->position, req->velocity, req->acceleration,
                      WriteOptions{WaitMode::None});  // 或 ByTime/ByReply 视协议而定
      resp->ok = true;
      resp->message = "command sent";
    } catch (const std::exception& e) {
      resp->ok = false;
      resp->message = e.what();
    }
  }

private:
  // params
  std::string port_;
  int baud_;
  std::vector<int64_t> ids_param_;
  std::vector<uint8_t> ids_;
  double pub_rate_hz_;
  std::string js_topic_, frame_id_;
  double resolution_ticks_;
  std::vector<double> gear_, sign_, offset_;

  // ros
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState js_msg_;
  rclcpp::Service<hklrc_motor_loop::srv::SetGripperPosition>::SharedPtr srv_;
  rclcpp::CallbackGroup::SharedPtr cg_;  // 互斥回调组（Humble可用）

  // device
  GripperMotor motor_;
  std::mutex io_mtx_;  // 保护读写（半双工安全）
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperDriverNode>());  // 单线程即可
  rclcpp::shutdown();
  return 0;
}
