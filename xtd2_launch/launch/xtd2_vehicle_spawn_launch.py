from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ros_gz_bridge.actions import RosGzBridge




def generate_launch_description():
    world_name_arg = DeclareLaunchArgument('world_name', description='Name of the world to launch (without .sdf)')
    model_name_arg = DeclareLaunchArgument('model', description='Name of the model to spawn')
    id_arg = DeclareLaunchArgument('id', description='ID of the model to spawn')
    name_space_arg = DeclareLaunchArgument('namespace', description='ROS namespace for the model')

    x_arg = DeclareLaunchArgument('x', default_value='0', description='X position of the model to spawn')
    y_arg = DeclareLaunchArgument('y', default_value='0', description='Y position of the model to spawn')
    z_arg = DeclareLaunchArgument('z', default_value='0', description='Z position of the model to spawn')
    roll_arg = DeclareLaunchArgument('roll', default_value='0', description='Roll angle of the model to spawn')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0', description='Pitch angle of the model to spawn')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0', description='Yaw angle of the model to spawn')


    # Model spawn and PX4 SITL
    px4_launch = Node(
        package='xtd2_launch',
        executable='px4_launch',
        arguments=[
            '--world', LaunchConfiguration('world_name'),
            '--model', LaunchConfiguration('model'),
            '--id', LaunchConfiguration('id'),
            '--namespace', LaunchConfiguration('namespace'),
            '--x', LaunchConfiguration('x'),
            '--y', LaunchConfiguration('y'),
            '--z', LaunchConfiguration('z'),
            '--roll', LaunchConfiguration('roll'),
            '--pitch', LaunchConfiguration('pitch'),
            '--yaw', LaunchConfiguration('yaw')
        ],
        output='screen',
        shell=True
    )

    # Gazebo-ROS2 Bridge 
    bridge_launch = Node(
        package='xtd2_launch',
        executable='bridge_launch',
        output='screen',
        arguments=[
            "--ros_ns", LaunchConfiguration('namespace'),
            "--gz_ns", LaunchConfiguration('namespace')
        ]
    )

    # XTDrone2 Communication
    xtd2_launch = Node(
        package='xtd2_communication',
        executable='multirotor_communication',
        output='screen',
        shell=True,
        arguments=[
            "--model", LaunchConfiguration('model'),
            "--id", LaunchConfiguration('id'),
            "--namespace", LaunchConfiguration('namespace')
        ]
    )

    ld = LaunchDescription([
        world_name_arg,
        model_name_arg,
        id_arg,
        name_space_arg,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        px4_launch,
        bridge_launch,
        xtd2_launch,
    ])

    return ld