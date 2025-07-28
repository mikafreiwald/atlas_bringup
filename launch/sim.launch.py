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

    # single robot:
    #   - warehouse: 0.15
    #   - marsyard2022: 1.75
    #   - rubicon: -11.83 -0.1 4.4  |  -7.0 0.0 3.96

    # dual robot:
    #   - warehouse:
    #      - atlas : 0, 0, 0.15
    #      - bestla: 0, 1.5, 0.15
    #   - marsyard2022:
    #      - atlas : -17.42, 8.14, 1.5
    #      - bestla: -16.52 -11.96 1.8

    # sdf file needs to have <world name="marsyard2020"> for mrg_slam to work
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_share_dir, 'launch', 'gz.launch.py'
        )]),
        launch_arguments={
            'sdf_file': os.path.join(package_share_dir, 'worlds', 'warehouse.sdf')
        }.items()
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_share_dir,'launch','rsp.launch.py'
        )]), launch_arguments={
            'use_sim_time': 'true'
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

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={('/cmd_vel_out', '/atlas/cmd_vel')},
        ros_arguments=['--log-level', 'INFO'],
        parameters=[
            {'use_sim_time': True, 'use_stamped': False},
            os.path.join(package_share_dir, 'config', 'twist_mux_topics.yaml'),
        ],
    )

    return LaunchDescription([
        gz_sim,
        rsp,
        # ad_ros2_mapping,
        # rviz,
        twist_mux,
    ])
