import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml_config(file_path):
    try:
        with open(file_path, "r") as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading config file: {e}")
        return None


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)

    config = load_yaml_config(config_file)

    nodes_to_start = []

    # Gazebo Simulation
    world_launch = Node(
        package='xtdrone2',
        executable='gazebo_launch',
        arguments=['--world', config['world']['name']],
    )
    nodes_to_start.append(world_launch)

    # xrce_dds_agent
    xrce_dds_process = ExecuteProcess(
        cmd=["MicroXRCEAgent udp4 -p 8888"],
        output='screen',
        name='microxrceagent',
        shell=True
    )
    nodes_to_start.append(xrce_dds_process)


    # Vehicles Setup
    for vehicle in config['vehicle']:
        # Spawn vehicle and PX4 SITL
        vehicle_node = Node(
            package='xtdrone2',
            executable='px4_launch',
            arguments=[
                '--model', vehicle['model'],
                '--id', str(vehicle['id']),
                '--world', config['world']['name'],
                '--x', str(vehicle['pose'][0]),
                '--y', str(vehicle['pose'][1]),
                '--z', str(vehicle['pose'][2]),
                '--roll', str(vehicle['pose'][3]),
                '--pitch', str(vehicle['pose'][4]),
                '--yaw', str(vehicle['pose'][5]),
            ],
            output='screen',
            shell=False
        )
        nodes_to_start.append(vehicle_node)

        # XTDrone2 Communication
        xtd2_launch = Node(
            package='xtdrone2',
            executable='multirotor_communication',
            output='screen',
            shell=True,
            arguments=[
                "--vehicle_type", vehicle['model'], 
                "--vehicle_id", str(vehicle['id']),
            ]
        )
        nodes_to_start.append(xtd2_launch)
    

    return nodes_to_start


def generate_launch_description():
    default_config_path = PathJoinSubstitution(
        [FindPackageShare("xtdrone2"), "launch", "launch_config", "default.yaml"]
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_path,
        description="Path to the configuration YAML file",
    )

    ld = LaunchDescription([config_file_arg])

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld