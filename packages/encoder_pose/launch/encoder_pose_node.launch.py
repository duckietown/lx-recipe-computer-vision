import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    veh = LaunchConfiguration("veh")
    config = os.path.join(get_package_share_directory("encoder_pose"), "config", "encoder_pose_node", "default.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("veh", default_value=EnvironmentVariable("VEHICLE_NAME"),
                              description="robot name (namespace of every topic)"),
        Node(
            package="encoder_pose",
            executable="encoder_pose_node",
            name="encoder_pose_node",
            namespace=veh,
            parameters=[config],
            output="screen",
            emulate_tty=True,
        ),
    ])
