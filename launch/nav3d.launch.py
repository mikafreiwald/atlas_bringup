from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace, SetParameter


def generate_launch_description():

    return LaunchDescription([

        SetParameter(name='use_sim_time', value=False),
        PushRosNamespace('/atlas'),

        Node(
            package='nav3d_ros2',
            executable='map_creator',
            parameters=[{
                "grid_size": 0.375,
                "gap_size": 1.2
            }],
            remappings=[('cloud', 'mrg_slam/map_points_service')],
            output="screen"
        ),

        Node(
            package='nav3d_ros2',
            executable='path_planner',
            parameters=[{
                "robot_frame": "atlas/velodyne",
                "robot_radius": 0.7,
                "min_turn_radius": 0.0,
                "goal_z_offset": 5.0,
            }],
            output="screen"
        ),

        Node(
            package='nav3d_ros2',
            executable='path_tracker',
            parameters=[{
                "robot_frame": "atlas/velodyne",
                "map_frame": "atlas/map",
                "min_target_distance": 0.5,
                "min_sub_target_distance": 1.0,
                "velocity": 0.5,
                'turn_rate': 1.0
            }],
            remappings=[('cmd_vel', 'cmd_vel')],
            output="screen"
        )
    ])
