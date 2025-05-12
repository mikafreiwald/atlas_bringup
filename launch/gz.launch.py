import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_share_dir = get_package_share_directory('atlas_bringup')
    ros_gz_sim_share_dir = get_package_share_directory('ros_gz_sim')

    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(package_share_dir, 'worlds'),
        ])
    )

    sdf_file = LaunchConfiguration('sdf_file')
    sdf_file_arg = DeclareLaunchArgument(
        'sdf_file',
        default_value=os.path.join(package_share_dir, 'worlds', 'marsyard2021.sdf'),
        description='Relative path to the SDF file within the worlds directory'
    )

    gz_args = ['-v 4 ', sdf_file]
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    return LaunchDescription([
        gz_resource_path,
        sdf_file_arg,
        gz_sim,
        LogInfo(msg='gz_args: '),
        LogInfo(msg=gz_args)
    ])
