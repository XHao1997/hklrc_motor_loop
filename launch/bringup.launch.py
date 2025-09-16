from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def make_urdf(serial_port, baud, ids):
    return f"""
<robot name="gripper">
  <link name="base"/>
  <link name="gripper_link"/>

  <joint name="gripper_joint" type="revolute">
    <parent link="base"/>
    <child  link="gripper_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="6.283185307" velocity="100.0" effort="10.0"/>
  </joint>

  <ros2_control name="SCServoHW" type="system">
    <hardware>
      <plugin>ftservo/SCServoSystem</plugin>
      <param name="serial_port">{serial_port}</param>
      <param name="baudrate">{baud}</param>
      <param name="ids">{ids}</param>
      <param name="position_ticks_per_rev">4096</param>
      <param name="has_position_interface">true</param>
      <param name="has_velocity_interface">true</param>
    </hardware>

    <joint name="gripper_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
"""

def launch_setup(context, *args, **kwargs):
    serial = LaunchConfiguration("serial_port").perform(context)
    baud   = LaunchConfiguration("baud").perform(context)
    ids    = LaunchConfiguration("ids").perform(context)

    robot_description = make_urdf(serial, baud, ids)

    pkg = get_package_share_directory("hklrc_motor_loop")
    controllers_yaml = os.path.join(pkg, "config", "controllers.yaml")

    # ✅ Publish robot_description on a topic for other nodes (TF, rviz, etc.)
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # ✅ ros2_control_node uses the same parameter
    cm = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers_yaml],
        output="screen",
    )

    # spawn controllers with a small delay (so controller_manager is ready)
    jsb = TimerAction(
        period=1.0,
        actions=[Node(
            package="controller_manager", executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager-timeout", "30"],
            output="screen"
        )]
    )
    vel = TimerAction(
        period=1.5,
        actions=[Node(
            package="controller_manager", executable="spawner",
            arguments=["vel_controller", "--controller-manager-timeout", "30"],
            output="screen"
        )]
    )

    return [rsp, cm, jsb, vel]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud", default_value="1000000"),
        DeclareLaunchArgument("ids", default_value="1"),
        OpaqueFunction(function=launch_setup),
    ])
