cmake_minimum_required(VERSION 3.16)
project(hklrc_motor_loop LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_subdirectory(${CMAKE_SOURCE_DIR}/../hklrc_freetech_motor/FTServo_Linux-main gripper_motor_build)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# If you also need your motor lib:
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGripperPosition.srv"
 DEPENDENCIES builtin_interfaces 
)
# 声明你的 srv 接口

add_executable(motor_loop src/motor_loop.cpp)
ament_target_dependencies(motor_loop rclcpp sensor_msgs)


# If you also use your motor lib in test.cpp:
target_link_libraries(motor_loop gripper::motor)
# 关键：把生成的头文件目录加入 include
rosidl_target_interfaces(motor_loop ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS motor_loop DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
ament_package()

