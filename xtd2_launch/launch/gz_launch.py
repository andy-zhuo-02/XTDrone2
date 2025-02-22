from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                            IncludeLaunchDescription, SetLaunchConfiguration, LogInfo)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_xtdrone2_gz_sim = get_package_share_directory('xtd2_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gz_model_path = PathJoinSubstitution([pkg_xtdrone2_gz_sim, 'models'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='aruco',
            description='World to load into Gazebo'
        ),
        SetLaunchConfiguration(name='world_file', 
                               value=[LaunchConfiguration('world'), 
                                      TextSubstitution(text='.sdf')]),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_model_path),
        LogInfo(msg=[TextSubstitution(text='Gazebo simulation resources path: '), gz_model_path]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': ['-r', ' ', PathJoinSubstitution([pkg_xtdrone2_gz_sim, 'worlds', LaunchConfiguration('world_file')])],
                'on_exit_shutdown': 'True',
            }.items(),
        ),
    ])