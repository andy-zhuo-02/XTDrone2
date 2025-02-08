from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xtd2_launch'

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
    maintainer_email='zhuoan@stu.pku.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_launch = xtd2_launch.utils.gazebo_launch:main',
            'px4_launch = xtd2_launch.utils.px4_launch:main',
        ],
    },
)
