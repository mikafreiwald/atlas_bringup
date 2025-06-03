import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_share_dir = get_package_share_directory('atlas_bringup')

    # single robot:
    #   - warehouse: 0.15
    #   - marsyard2022: 1.75

    # dual robot:
    #   - marsyard2022:
    #      - atlas : -17.42, 8.14, 1.5
    #      - bestla: -16.52 -11.96 1.8

    # sdf file needs to have <world name="marsyard2020"> for mrg_slam to work
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_share_dir, 'launch', 'gz.launch.py'
        )]),
        launch_arguments={
            'sdf_file': os.path.join(package_share_dir, 'worlds', 'marsyard2022.sdf')
        }.items()
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_share_dir,'launch','rsp.launch.py'
        )]), launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    mrg_slam_sim_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrg_slam_sim'), 'launch', 'dual_robot_sim.launch.py'
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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(package_share_dir, 'config', 'atlas.rviz')]
    )

    ad_ros2_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ad_ros2'), 'launch', 'local_elevation_mapping.launch.py'
        )])
    )

    return LaunchDescription([
        gz_sim,
        rsp,
        mrg_slam_sim_robot,
        mrg_slam,
        ad_ros2_mapping,
        rviz,
    ])
