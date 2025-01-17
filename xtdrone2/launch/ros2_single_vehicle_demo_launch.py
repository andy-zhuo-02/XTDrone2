from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world_name_arg = DeclareLaunchArgument('world_name', default_value='aruco', description='Name of the world to launch (without .sdf)')
    
    #####################
    # Gazebo Simulation #
    #####################
    world_launch = Node(
        package='xtdrone2',
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

    ##################
    # XRCE-DDS Agent #
    ##################
    # TODO

    ############################
    # Model spawn and PX4 SITL #
    ############################
    px4_launch = Node(
        package='xtdrone2',
        executable='px4_launch',
        arguments=[
            '--model', 'gz_x500_depth',
            '--id', '0',
            '--world', LaunchConfiguration('world_name'),
        ],
        output='screen',
        shell=True
    )

    # Add all nodes to the launch description
    ld = LaunchDescription([
        world_name_arg,
        world_launch,
        px4_launch,
    ])

    return ld