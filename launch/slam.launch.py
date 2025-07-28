import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    dual_robot = LaunchConfiguration('dual_robot')

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

    bestla_keyboard = Node(
        condition=IfCondition(dual_robot),
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard_node_bestla',
        remappings=[('/cmd_vel', '/bestla/cmd_vel')],
        output='screen',
        prefix='gnome-terminal -- ',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'dual_robot',
            default_value='false',
            description='Use dual (true) or single (false) robot simulation',
        ),

        mrg_slam_sim_robot_single,
        mrg_slam_sim_robot_dual,
        mrg_slam,
        bestla_keyboard,
    ])
