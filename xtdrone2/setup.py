import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'xtdrone2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch', 'launch_config'), glob(os.path.join('launch', 'launch_config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andy-station-ubuntu-24',
    maintainer_email='andy-station-ubuntu-24@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_control_demo = xtdrone2.offboard_control_demo:main',
            'multirotor_communication = xtdrone2.communication.multirotor_communication:main',
            'multirotor_keyboard_control = xtdrone2.keyboard_control.multirotor_keyboard_control:main',
            'gazebo_launch = xtdrone2.bringup.gazebo_launch:main',
            'px4_launch = xtdrone2.bringup.px4_launch:main',
        ],
    },
)
