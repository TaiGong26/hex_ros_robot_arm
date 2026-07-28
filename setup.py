import os
from setuptools import setup, find_packages
from glob import glob

package_name = 'hex_ros_robot_arm'


def get_files(tar: str, src: str):
    all_paths = glob(f'{src}/*')

    data_files = []
    for path in all_paths:
        if os.path.isfile(path):
            data_files.append((tar, [path]))
        elif os.path.isdir(path):
            sub_files = get_files(f'{tar}/{os.path.basename(path)}', path)
            data_files.extend(sub_files)

    return data_files


setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *get_files('share/' + package_name, 'launch/ros2'),
        *get_files('share/' + package_name, 'launch/ros1'),
        *get_files('share/' + package_name + '/config/ros2', 'config/ros2'),
        *get_files('share/' + package_name + '/config/ros1', 'config/ros1'),
        # *get_files('share/' + package_name, "launch/ros2"),
        # *get_files('share/' + package_name + '/config/ros2', "config/ros2"),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hexfellow',
    maintainer_email='thetaigon@qq.com',
    description='ROS package with separated robot nodes for archer/firefly/hello',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # 'robot_archer_y6 = hex_ros_robot_arm.robot_archer_y6:main',
            # 'robot_firefly_y6 = hex_ros_robot_arm.robot_firefly_y6:main',
            # 'robot_hello_y6 = hex_ros_robot_arm.robot_hello_y6:main',
            'hex_ros_robot_archer_y6 = hex_ros_robot_arm.robot_archer_y6:main',
            'hex_ros_robot_firefly_y6 = hex_ros_robot_arm.robot_firefly_y6:main',
            'hex_ros_robot_hello_y6 = hex_ros_robot_arm.robot_hello_y6:main',
            'test_ctrl = hex_ros_robot_arm.test_ctrl:main',
        ],
    },
)
