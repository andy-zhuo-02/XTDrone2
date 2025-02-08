from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    world_name_arg = DeclareLaunchArgument('world_name', default_value='aruco', description='Name of the world to launch (without .sdf)')
    
    #####################
    # Gazebo Simulation #
    #####################
    world_launch = Node(
        package='xtd2_launch',
        executable='gazebo_launch',
        arguments=[
            '--world', LaunchConfiguration('world_name'),
            # '--model-store', LaunchConfiguration('resource_store_path')
        ],  
    )

    ######################
    # Gazebo-ROS2 Bridge #
    ######################
    # TODO

    ############################
    # Model spawn and PX4 SITL #
    ############################
    px4_launch = Node(
        package='xtd2_launch',
        executable='px4_launch',
        arguments=[
            '--model', 'gz_x500',
            '--id', '0',
            '--world', LaunchConfiguration('world_name'),
        ],
        output='screen',
        shell=True
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

    ##########################
    # XTDrone2 Communication #
    ##########################
    xtd2_launch = Node(
        package='xtd2_communication',
        executable='multirotor_communication',
        output='screen',
        shell=True,
        arguments=[
            "--vehicle_type", "gz_x500", 
            "--vehicle_id", "0"
        ]
    )

    # Add all nodes to the launch description
    ld = LaunchDescription([
        world_name_arg,
        world_launch,
        px4_launch,
        xrce_dds_process,
        xtd2_launch
    ])

    return ld