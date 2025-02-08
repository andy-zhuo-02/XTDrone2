from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world_name_arg = DeclareLaunchArgument('world_name', default_value='aruco', description='Name of the world to launch (without .sdf)')
    
    world_launch = Node(
        package='xtd2_launch',
        executable='gazebo_launch',
        arguments=[
            '--world', LaunchConfiguration('world_name'),
            # '--model-store', LaunchConfiguration('resource_store_path')
        ],  
    )

    ld = LaunchDescription([
        world_name_arg,
        world_launch,
    ])

    for i in range(4):
        px4_launch = Node(
            package='xtd2_launch',
            executable='px4_launch',
            arguments=[
                '--model', 'gz_x500',
                '--id', str(i),
                '--world', LaunchConfiguration('world_name'),
                '--x', str(i*2),
                '--y', '0',
            ],
            output='screen',
            shell=True
        )
        ld.add_action(px4_launch)

    return ld