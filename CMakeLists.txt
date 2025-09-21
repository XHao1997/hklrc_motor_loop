cmake_minimum_required(VERSION 3.16)
project(hklrc_motor_loop LANGUAGES C CXX)

# ---- C++ standard ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ROS 2 interfaces
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Your motor SDK (keep your existing dependency)
find_package(FTServo REQUIRED)  # provides BerryGripper::gripper_motor (per your setup)

# ros2_control hardware plugin deps
find_package(ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# ---- ROSIDL: service generation ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGripperPosition.srv"
  DEPENDENCIES builtin_interfaces
)
# For consumers of generated headers
ament_export_dependencies(rosidl_default_runtime)

# ======================================================================
# 1) ros2_control Hardware Plugin library: berry_gripper_system
#    - headers in include/berry_gripper/gripper_system.hpp
#    - source in  src/gripper_system.cpp
#    - plugin XML in gripper_system_plugin.xml
# ======================================================================
add_library(berry_gripper_system SHARED
  src/hklrc_motor_hardware.cpp
)

target_include_directories(berry_gripper_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(berry_gripper_system
  hardware_interface
  pluginlib
  rclcpp           # used for logging
  ros2_control
)

# Link your SDK if needed by the hardware implementation
target_link_libraries(berry_gripper_system
  BerryGripper::gripper_motor
)

# Export plugin description (lets pluginlib discover it)
pluginlib_export_plugin_description_file(hardware_interface gripper_system_plugin.xml)

# Install plugin library, headers, and XML
install(TARGETS berry_gripper_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES gripper_system_plugin.xml DESTINATION share/${PROJECT_NAME})

# ======================================================================
# 2) Your existing node (motor_loop) — kept as-is
# ======================================================================
add_executable(motor_loop src/motor_loop.cpp)
ament_target_dependencies(motor_loop rclcpp sensor_msgs)

# Use PLAIN signature (ok)
target_link_libraries(motor_loop BerryGripper::gripper_motor)

# Link the C++ typesupport for your generated service
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(motor_loop ${cpp_typesupport_target})

# ---- Install runtime assets ----
install(TARGETS motor_loop DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY urdf/   DESTINATION share/${PROJECT_NAME}/urdf)


# ---- Export package ----
ament_export_dependencies(
  rclcpp
  sensor_msgs
  hardware_interface
  pluginlib
  ros2_control
  rosidl_default_runtime
)

ament_package()
