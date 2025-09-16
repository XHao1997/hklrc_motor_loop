cmake_minimum_required(VERSION 3.16)
project(hklrc_motor_loop LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)   # runtime for generated interfaces
find_package(FTServo REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGripperPosition.srv"
  DEPENDENCIES builtin_interfaces
)

add_executable(motor_loop src/motor_loop.cpp)
ament_target_dependencies(motor_loop rclcpp sensor_msgs)

# Use PLAIN signature (no PUBLIC/PRIVATE/INTERFACE)
target_link_libraries(motor_loop BerryGripper::gripper_motor)

# Attach generated ROSIDL interfaces (also uses PLAIN signature internally)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(motor_loop ${cpp_typesupport_target})
# (optional but helpful)
# target_include_directories(motor_loop PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
#   $<INSTALL_INTERFACE:include>
# )
# ---- Install ----
install(TARGETS motor_loop DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
