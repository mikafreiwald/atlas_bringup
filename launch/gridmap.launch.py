import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_share_dir = get_package_share_directory('atlas_bringup')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_share_dir, 'launch', 'gz.launch.py'
        )]),
        launch_arguments={
            'sdf_file': os.path.join(package_share_dir, 'worlds', 'gridmap.sdf')
        }.items()
    )

    return LaunchDescription([
        gz_sim,
    ])
