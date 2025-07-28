from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node, SetParameter, PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        PushRosNamespace(EnvironmentVariable(name='ROBOT_NAME', default_value='atlas')),
        SetParameter('use_sim_time', 'True'),
        Node(
            package='frontier_exploration',
            executable='classical_frontier_detector', # same as cmake
            name='classical_frontier_detector',
            # prefix='valgrind --leak-check=yes ', # Uncomment to run valgrind. Requires debug build
            output='screen',
            parameters=[
                {"region_size_thresh": 12}, # number of points
                {"robot_width": 0.5}, # meters
                {"occupancy_map_msg": "global_costmap/costmap"} # occupany grid map topic
            ]
        ),
     ])