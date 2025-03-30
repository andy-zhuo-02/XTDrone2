"""
XTDrone2 Bridge Launch File

Author: Andy Zhuo
Email: zhuoan@stu.pku.edu.cn

Copyright (c) 2025 XTDrone2

This file is part of XTDrone2. XTDrone2 is free software: you can redistribute it
and/or modify it under the terms of the MIT License.

You should have received a copy of the MIT License along with XTDrone2. If not,
see <https://opensource.org/licenses/MIT>.
"""

import os
import yaml
import argparse
import subprocess

from string import Template

from launch_ros.substitutions import FindPackageShare


def main():
    parser = argparse.ArgumentParser(description='Launch Gazebo ROS bridge in XTDrone2')

    parser.add_argument('--ros_ns', type=str, help='ROS namespace', required=True)
    parser.add_argument('--gz_ns', type=str, help='Gazebo namespace', required=True)

    args, unknown = parser.parse_known_args()

    # 通过YAML模板设定每个vehicle的桥接设置
    yaml_template = os.path.join(FindPackageShare("xtd2_launch").find('xtd2_launch'), 'launch', 'launch_config', 'ros_gz_bridge.yaml')
    yaml_tmp_output = os.path.join(FindPackageShare("xtd2_launch").find('xtd2_launch'), 'launch', 'launch_config', f'tmp_ros_gz_bridge_{args.ros_ns}.yaml')
    with open(yaml_template, 'r') as file:
        yaml_template_content = file.read()
    template = Template(yaml_template_content).safe_substitute({'gz_ns': args.gz_ns, 'ros_ns': args.ros_ns})
    with open(yaml_tmp_output, 'w') as file:
        yaml.safe_dump(yaml.safe_load(template), file)


    pose_bridge_cmd = f'ros2 launch ros_gz_bridge ros_gz_bridge.launch.py bridge_name:=ros_gz_bridge_{args.ros_ns} config_file:={yaml_tmp_output}'

    _handle = subprocess.Popen(['bash', '-c', pose_bridge_cmd])

    try:
        while True:
            if _handle.poll() is not None:
                break
            else:
                _handle.wait()
    except KeyboardInterrupt:
        _handle.terminate()
        exit(0)



if __name__ == '__main__':
    main()