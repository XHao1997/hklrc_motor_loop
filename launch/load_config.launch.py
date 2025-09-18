from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("hklrc_motor_loop")
    params_file = os.path.join(pkg_share, "config", "motor_config.yaml")
    print("praram file: ", params_file)
    return LaunchDescription([
        Node(
            package="hklrc_motor_loop",
            executable="motor_loop",
            name="gripper_driver_node",
            parameters=[params_file],
            output="screen",
        )
    ])
 