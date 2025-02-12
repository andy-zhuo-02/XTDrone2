from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution




def generate_launch_description():
    world_name_arg = DeclareLaunchArgument('world_name', default_value='aruco', description='Name of the world to launch (without .sdf)')
    

    # Gazebo Simulation 
    world_launch = Node(
        package='xtd2_launch',
        executable='gazebo_launch',
        arguments=[
            '--world', LaunchConfiguration('world_name'),
            # '--model-store', LaunchConfiguration('resource_store_path')
        ],  
    )

    # XRCE-DDS Agent 
    xrce_dds_process = ExecuteProcess(
        cmd=["MicroXRCEAgent udp4 -p 8888"],
        output='screen',
        name='microxrceagent',
        shell=True
    )


    ld = LaunchDescription([
        world_name_arg,
        world_launch,
        xrce_dds_process,
    ])

    for i in range(4):
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
                'model': 'gz_x500',
                'id': TextSubstitution(text=str(i)),
                'namespace': TextSubstitution(text=f'x500_{i}'),
                'x': TextSubstitution(text=str(i*2)),
                'y': '0',
                'z': '0',
                'roll': '0',
                'pitch': '0',
                'yaw': '0',
            }.items()
        )

        ld.add_action(spawn)

    return ld