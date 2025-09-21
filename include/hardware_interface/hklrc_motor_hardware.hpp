#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "gripper_motor/GripperMotor.h"

namespace berrygripper_hw
{
class GripperSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GripperSystem)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // params from URDF ros2_control > hardware > params
  std::string port_;
  int baud_{1'000'000};
  int id_{1};
  double resolution_ticks_{4096.0};
  double gear_{1.0};
  double sign_{+1.0};
  double offset_rad_{0.0};
  int max_torque_{1000};

  // state/command (radians)
  double pos_{0.0}, vel_{0.0};
  double cmd_pos_{NAN};  // position command (rad)

  // device
  berrygripper::GripperMotor motor_;
  bool opened_{false};

  // helpers
  double ticks_to_rad(int32_t ticks) const;
  double ticksps_to_radps(int32_t v) const;
  int32_t rad_to_ticks(double rad) const;
};

}  // namespace berrygripper_hw
