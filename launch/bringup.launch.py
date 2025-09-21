# launch/bringup.launch.py  (ROS 2 Jazzy)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ---- Declare args (和你给的模板一致的风格) ----
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="hklrc_motor_loop",
            description="Package that contains the URDF/Xacro of the gripper."
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="gripper.urdf.xacro",
            description="URDF/Xacro file of the gripper."
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="config/controllers.yaml",
            description="Controller Manager parameters YAML."
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Launch RViz2 and Joint State Publisher GUI."
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Optional joint name prefix (multi-robot setups)."
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    description_file    = LaunchConfiguration("description_file")
    controllers_file    = LaunchConfiguration("controllers_file")
    gui                 = LaunchConfiguration("gui")
    prefix              = LaunchConfiguration("prefix")

    # ---- Build paths ----
    pkg_share = FindPackageShare(description_package)
    urdf_path = PathJoinSubstitution([pkg_share, "urdf", description_file])
    controllers_path = PathJoinSubstitution([pkg_share, controllers_file])

    # ---- robot_description via xacro (Jazzy 要求经 topic 提供) ----
    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"),
            " ",
            urdf_path,
            " ",
            "prefix:=", prefix,
        ])
    }
    # ---- Nodes ----
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],   # 在 Jazzy 中必须经 topic 发布
        output="screen",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_path],    # 不要再传 robot_description 给这个节点
        output="screen",
    )

    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawner_pos = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["vel_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 可选 RViz（如果你有 rviz 配置也可放这里）
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(gui),
        output="log",
    )

    nodes = [
        joint_state_publisher_gui,
        robot_state_publisher,
        controller_manager,
        spawner_jsb,
        spawner_pos,
        rviz,
    ]

    return LaunchDescription(declared_arguments + nodes)
