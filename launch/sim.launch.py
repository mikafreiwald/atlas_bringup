import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # mrg_slam_sim_world = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('mrg_slam_sim'), 'launch', 'marsyard2020.launch.py'
    #     )])
    # )

    mrg_slam_sim_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('atlas_bringup'), 'launch', 'marsyard2022.launch.py'
        )])
    )

    mrg_slam_sim_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrg_slam_sim'), 'launch', 'single_robot_sim.launch.py'
        )])
    )

    mrg_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrg_slam'), 'launch', 'mrg_slam.launch.py'
        )]),
        launch_arguments={
            'init_odom_topic': '/atlas/odom_ground_truth',
        }.items()
    )

    ad_ros2_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ad_ros2'), 'launch', 'local_elevation_mapping.launch.py'
        )])
    )

    return LaunchDescription([
        mrg_slam_sim_world,
        mrg_slam_sim_robot,
        mrg_slam,
        ad_ros2_mapping
    ])