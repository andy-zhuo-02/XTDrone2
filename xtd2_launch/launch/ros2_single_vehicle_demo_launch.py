from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    world_name_arg = DeclareLaunchArgument('world_name', default_value='aruco', description='Name of the world to launch (without .sdf)')
    model_name_arg = DeclareLaunchArgument('model_name', default_value='gz_x500', description='Name of the model to spawn')
    id_arg = DeclareLaunchArgument('id', default_value='0', description='ID of the model to spawn')
    name_space_arg = DeclareLaunchArgument('namespace', default_value='x500_0', description='ROS namespace for the model')
    
    #####################
    # Gazebo Simulation #
    #####################
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("xtd2_launch"),
                "launch",
                "gz_launch.py"
            ])
        ]),
        launch_arguments={
            "world": LaunchConfiguration('world_name'),
        }.items()
    )

    ##################
    # XRCE-DDS Agent #
    ##################
    xrce_dds_process = ExecuteProcess(
        cmd=["MicroXRCEAgent udp4 -p 8888"],
        output='screen',
        name='microxrceagent',
        shell=True
    )

    ###################################################################
    # Spawn vehicles, including Model, PX4 SITL and ROS-Gazebo bridge #
    ###################################################################
    spawn = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('xtd2_launch'),
                    'launch',
                    'xtd2_vehicle_spawn_launch.py'
                ])
            ]),
            launch_arguments={
                'world_name': LaunchConfiguration('world_name'),
                'model': 'r1_rover',
                'id': '0',
                'namespace': 'r1_rover_0',
            }.items()
        )

    # Add all nodes to the launch description
    ld = LaunchDescription([
        world_name_arg,
        model_name_arg,
        id_arg,
        name_space_arg,
        world_launch,
        xrce_dds_process,
        spawn
    ])

    return ld