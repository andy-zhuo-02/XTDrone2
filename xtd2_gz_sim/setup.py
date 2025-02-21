import os
from setuptools import find_packages, setup

package_name = 'xtd2_gz_sim'

def get_data_files(directory):
    data_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            file_path = os.path.join(root, file)
            install_path = os.path.join('share', package_name, directory, os.path.relpath(root, directory))
            data_files.append((install_path, [file_path]))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + get_data_files('models') + get_data_files('worlds'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haoxin',
    maintainer_email='139300141+ha0xin@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)