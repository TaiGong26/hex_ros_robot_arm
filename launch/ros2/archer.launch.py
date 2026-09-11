#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "hex_ros_robot_arm"

    # args
    test_arg = DeclareLaunchArgument(
        name='test',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to turn on test ctrl node')
    robot_host_arg = DeclareLaunchArgument(
        name='robot_host',
        default_value='192.168.1.100',
        description='Robot controller IP address')
    robot_port_arg = DeclareLaunchArgument(
        name='robot_port',
        default_value='8439',
        description='Robot controller WebSocket port')
    robot_grip_type_arg = DeclareLaunchArgument(
        name='robot_grip_type',
        default_value='empty',
        choices=['gp100','gp80','gr100', 'empty'],
        description='Grip type: gp100/gp80/gr100 (1-DoF) or empty (0-DoF)')
    
    variant_arg = DeclareLaunchArgument(
        name='variant',
        default_value='archer',
        choices=['archer'],
        description='Robot variant: archer or')


    # robot node
    robot_param_path = FindPackageShare(package_name).find(
        package_name) + '/config/ros2/archer_params.yaml'
    robot_node = Node(
        package=package_name,
        executable='hex_ros_robot_archer_y6',
        name='hex_ros_robot_archer_y6',
        output="screen",
        emulate_tty=True,
        parameters=[
            robot_param_path,
            {
                'robot_host': LaunchConfiguration('robot_host'),
                'robot_port': LaunchConfiguration('robot_port'),
                'robot_grip_type':
                LaunchConfiguration('robot_grip_type'),
            },
        ],
        remappings=[
            ('manip_ctrl', 'manip_ctrl'),
            ('manip_state', 'manip_state'),
            ('joint_states', 'joint_states'),
        ]
    )

    # test group
    test_group = GroupAction(
        [
            Node(
                package=package_name,
                executable='test_ctrl',
                name='test_ctrl',
                output="screen",
                emulate_tty=True,
                parameters=[{
                    'use_sim_time': False,
                }],
                remappings=[
                    ('manip_ctrl', 'manip_ctrl'),
                ],
            )
        ],
        condition=IfCondition(LaunchConfiguration('test')),
    )

    return LaunchDescription([
        test_arg,
        robot_host_arg,
        robot_port_arg,
        robot_grip_type_arg,
        variant_arg,
        robot_node,
        test_group,
    ])
