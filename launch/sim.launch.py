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

    dual_robot = LaunchConfiguration('dual_robot')

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

    mrg_slam_sim_robot_single = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrg_slam_sim'), 'launch', 'single_robot_sim.launch.py'
        )]),
        launch_arguments={
            'world': 'rubicon',
        }.items(),
        condition=UnlessCondition(dual_robot),
    )

    mrg_slam_sim_robot_dual = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrg_slam_sim'), 'launch', 'dual_robot_sim.launch.py'
        )]),
        condition=IfCondition(dual_robot),
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

    bestla_keyboard = Node(
        condition=IfCondition(dual_robot),
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard_node_bestla',
        remappings=[('/cmd_vel', '/bestla/cmd_vel')],
        output='screen',
        prefix='gnome-terminal -- ',
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
        DeclareLaunchArgument(
            'dual_robot',
            default_value='false',
            description='Use dual (true) or single (false) robot simulation',
        ),

        gz_sim,
        rsp,
        mrg_slam_sim_robot_single,
        mrg_slam_sim_robot_dual,
        mrg_slam,
        # ad_ros2_mapping,
        rviz,
        bestla_keyboard,
        twist_mux,
    ])
