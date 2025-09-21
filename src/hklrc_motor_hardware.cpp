#include "hardware_interface/hklrc_motor_hardware.hpp"  // your header that declares berrygripper_hw::GripperSystem

#include <rclcpp/rclcpp.hpp>                 // logging/macros
#include <pluginlib/class_list_macros.hpp>   // PLUGINLIB_EXPORT_CLASS
#include <limits>
#include <map>
#include <vector>
#include <string>
#include <sstream>    // std::istringstream
#include <cmath>      // std::round, std::isfinite, M_PI

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using hardware_interface::SystemInterface;
using hardware_interface::StateInterface;
using hardware_interface::CommandInterface;

namespace berrygripper_hw
{

double GripperSystem::ticks_to_rad(int32_t ticks) const {
  const double rad = (static_cast<double>(ticks) / resolution_ticks_) * 2.0 * M_PI;
  return sign_ * rad * gear_ + offset_rad_;
}

double GripperSystem::ticksps_to_radps(int32_t v) const {
  const double radps = (static_cast<double>(v) / resolution_ticks_) * 2.0 * M_PI;
  return sign_ * radps * gear_;
}

int32_t GripperSystem::rad_to_ticks(double rad) const {
  const double base = (rad - offset_rad_) / (sign_ * gear_);
  return static_cast<int32_t>(std::round(base * (resolution_ticks_ / (2.0 * M_PI))));
}

CallbackReturn GripperSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  // Read params from <ros2_control> hardware params
  auto get = [&](const std::string & key, auto & out, auto def){
    if (info_.hardware_parameters.count(key)) {
      std::istringstream ss(info_.hardware_parameters.at(key));
      ss >> out;
    } else {
      out = def;
    }
  };

  get("serial_port",     port_,            std::string("/dev/ttyUSB3"));
  get("baud",            baud_,            1'000'000);
  get("id",              id_,              1);
  get("resolution_ticks",resolution_ticks_,4096.0);
  get("gear_ratio",      gear_,            1.0);
  get("sign",            sign_,            +1.0);
  get("offset_rad",      offset_rad_,      0.0);
  get("max_torque",      max_torque_,      1000);

  // one joint expected
  if (info_.joints.size() != 1) {
    RCLCPP_ERROR(rclcpp::get_logger("GripperSystem"),
                 "Expect exactly 1 joint, got %zu", info_.joints.size());
    return CallbackReturn::ERROR;
  }

  // Joint must have position state + position command
  const auto & j = info_.joints[0];
  if (j.state_interfaces.size() < 2 || j.command_interfaces.size() < 1 ||
      j.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
      j.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY ||
      j.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_ERROR(rclcpp::get_logger("GripperSystem"),
      "Joint must have state interfaces [position, velocity] and command interface [position]");
    return CallbackReturn::ERROR;
  }

  pos_ = 0.0;
  vel_ = 0.0;
  cmd_pos_ = std::numeric_limits<double>::quiet_NaN();

  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> GripperSystem::export_state_interfaces()
{
  std::vector<StateInterface> si;
  si.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &pos_);
  si.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &vel_);
  return si;
}

std::vector<CommandInterface> GripperSystem::export_command_interfaces()
{
  std::vector<CommandInterface> ci;
  ci.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &cmd_pos_);
  return ci;
}

CallbackReturn GripperSystem::on_activate(const rclcpp_lifecycle::State &)
{
  // open device
  if (!motor_.open(port_, baud_)) {
    RCLCPP_ERROR(rclcpp::get_logger("GripperSystem"), "Failed to open %s @ %d", port_.c_str(), baud_);
    return CallbackReturn::ERROR;
  }
  opened_ = true;

  // enable + torque limit
  motor_.switchTorque(static_cast<uint8_t>(id_), true);
  motor_.setLimitTorque(static_cast<uint8_t>(id_), max_torque_);

  // initialize state from hardware
  std::map<uint8_t, berrygripper::Feedback> fb;
  motor_.readPosSpeed({static_cast<uint8_t>(id_)}, fb);
  if (auto it = fb.find(static_cast<uint8_t>(id_)); it != fb.end()) {
    pos_ = ticks_to_rad(it->second.position);
    vel_ = ticksps_to_radps(it->second.speed);
  } else {
    pos_ = 0.0;
    vel_ = 0.0;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (opened_) {
    motor_.switchTorque(static_cast<uint8_t>(id_), false);
    motor_.close();
    opened_ = false;
  }
  return CallbackReturn::SUCCESS;
}

return_type GripperSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::map<uint8_t, berrygripper::Feedback> fb;
  if (!motor_.readPosSpeed({static_cast<uint8_t>(id_)}, fb)) {
    static rclcpp::Clock clock(RCL_STEADY_TIME);  // needed by *_THROTTLE macros
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("GripperSystem"), clock, 2000,
                         "readPosSpeed failed");
    return return_type::OK;
  }
  auto it = fb.find(static_cast<uint8_t>(id_));
  if (it != fb.end()) {
    pos_ = ticks_to_rad(it->second.position);
    vel_ = ticksps_to_radps(it->second.speed);
  }
  return return_type::OK;
}

return_type GripperSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!std::isfinite(cmd_pos_)) {
    return return_type::OK;
  }

  const int32_t ticks = rad_to_ticks(cmd_pos_);
  // You can add velocity/acc limits as params if needed
  const int32_t vel = 0;  // 0 -> use default/previous
  const int32_t acc = 0;
  try {
    motor_.setMode(static_cast<uint8_t>(id_), 0); // 0: servo mode
    motor_.writePos(static_cast<uint8_t>(id_), ticks, vel, acc,
                    berrygripper::WriteOptions{berrygripper::WaitMode::None});
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("GripperSystem"), "writePos exception");
  }
  return return_type::OK;
}

} // namespace berrygripper_hw

PLUGINLIB_EXPORT_CLASS(berrygripper_hw::GripperSystem, hardware_interface::SystemInterface)
