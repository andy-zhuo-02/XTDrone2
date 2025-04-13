from setuptools import find_packages, setup

package_name = 'xtd2_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'multirotor_keyboard_control = xtd2_control.keyboard.multirotor_keyboard_control:main',
            'rover_keyboard_control = xtd2_control.keyboard.rover_keyboard_control:main',
        ],
    },
)
