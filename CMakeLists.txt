cmake_minimum_required(VERSION 3.16)
project(hklrc_motor_loop LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# If you also need your motor lib:
find_package(FTServo CONFIG REQUIRED)

add_executable(motor_loop src/motor_loop.cpp)

# ✅ Link ROS 2 targets via ament helper (DON'T use target_link_libraries(motor_loop rclcpp))
ament_target_dependencies(motor_loop rclcpp sensor_msgs)

# If you also use your motor lib in test.cpp:
target_link_libraries(motor_loop FTServo::gripper_motor)

install(TARGETS motor_loop DESTINATION lib/${PROJECT_NAME})
ament_package()
