#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import numpy as np

import rospy

from hex_util_runtime import ns_now

from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import ColorRGBA
from hex_ros_msgs.msg import (
    HexRosRoboManipStateStamped,
    HexRosTeleopHandleStateStamped,
)

from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboManipStateStamped,
)
from hex_util_msg.dataclass.dataclass_teleop import (
    HexDcTeleopHandleStateStamped,
)

from .interface_base import HelloInterfaceBase
from .interface_base import JOINT_STATE_NAME


class DataInterface(HelloInterfaceBase):

    def __init__(self, name: str = "unknown"):
        rospy.init_node(name, anonymous=True)
        super().__init__(name)

        ### rate parameters
        self._rate_param["ros"] = rospy.get_param('~ctrl_rate', 500.0)
        self._rate_param["state"] = rospy.get_param('~rate_state', 100.0)
        self.__rate = rospy.Rate(self._rate_param["ros"])

        ### robot parameters (no grip_type — Hello Y6 has no gripper)
        self._robot_param = {
            "host": rospy.get_param('~robot_host', "192.168.1.100"),
            "port": rospy.get_param('~robot_port', 8439),
            "frame_id": rospy.get_param('~robot_frame_id', "base_link"),
            "state_buffer_size": rospy.get_param('~state_buffer_size', 200),
            "sens_ts": rospy.get_param('~sens_ts', True),
        }

        ### time source — PTP (ns_now) or ROS clock
        self._use_ros_time = rospy.get_param('~use_ros_time', False)

        ### publisher — manip_state
        self.__manip_state_pub = rospy.Publisher(
            'manip_state',
            HexRosRoboManipStateStamped,
            queue_size=10,
        )
        ### publisher — joint_states
        self.__joint_state_pub = rospy.Publisher(
            'joint_states',
            JointState,
            queue_size=10,
        )
        ### publisher — /clock
        self.__clock_pub = rospy.Publisher(
            '/clock',
            Clock,
            queue_size=10,
        )
        ### publisher — joy_state (Hello grip joy)
        self.__joy_state_pub = rospy.Publisher(
            'joy_state',
            HexRosTeleopHandleStateStamped,
            queue_size=10,
        )

        ### NOTE: No manip_ctrl subscriber — Hello Y6 is read-only.

        ### subscriber — color_cmd (RGB LED control, std_msgs/ColorRGBA)
        self.__color_cmd_sub = rospy.Subscriber(
            'color_cmd',
            ColorRGBA,
            self.__color_cmd_callback,
        )
        self.__color_cmd_sub

    def sleep(self):
        self.__rate.sleep()

    ####################
    ### ros infrastructure
    ####################
    def ok(self) -> bool:
        return not rospy.is_shutdown()

    def shutdown(self):
        pass

    ####################
    ### logging
    ####################
    def logd(self, msg, *args, **kwargs):
        rospy.logdebug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        rospy.loginfo(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        rospy.logwarn(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        rospy.logerr(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        rospy.logfatal(msg, *args, **kwargs)

    ####################
    ### time source
    ####################
    def now_ns(self) -> int:
        if self._use_ros_time:
            return rospy.Time.now().to_nsec()
        return ns_now()

    ####################
    ### publishers
    ####################
    def pub_manip_state(self, out: HexDcRoboManipStateStamped):
        msg = HexRosRoboManipStateStamped()
        msg.header.stamp = rospy.Time(
            int(out.header.stamp.secs),
            int(out.header.stamp.nsecs),
        )
        msg.header.frame_id = out.header.frame_id

        arm = out.manip_state.arm_state
        msg.manip_state.arm_state.jnt.position = \
            np.asarray(arm.jnt.position, dtype=np.float64).tolist()
        msg.manip_state.arm_state.jnt.velocity = \
            np.asarray(arm.jnt.velocity, dtype=np.float64).tolist()
        msg.manip_state.arm_state.jnt.effort = \
            np.asarray(arm.jnt.effort, dtype=np.float64).tolist()
        msg.manip_state.arm_state.pose.position.x = arm.pose.position.x
        msg.manip_state.arm_state.pose.position.y = arm.pose.position.y
        msg.manip_state.arm_state.pose.position.z = arm.pose.position.z
        msg.manip_state.arm_state.pose.orientation.x = arm.pose.orientation.x
        msg.manip_state.arm_state.pose.orientation.y = arm.pose.orientation.y
        msg.manip_state.arm_state.pose.orientation.z = arm.pose.orientation.z
        msg.manip_state.arm_state.pose.orientation.w = arm.pose.orientation.w

        # Hello Y6 has no gripper — publish empty grip state
        msg.manip_state.grip_state.jnt.position = []
        msg.manip_state.grip_state.jnt.velocity = []
        msg.manip_state.grip_state.jnt.effort = []

        self.__manip_state_pub.publish(msg)

    def pub_joint_state(self, out: HexDcRoboManipStateStamped):
        msg = JointState()
        msg.header.stamp = rospy.Time(
            int(out.header.stamp.secs),
            int(out.header.stamp.nsecs),
        )
        msg.header.frame_id = out.header.frame_id
        msg.name = JOINT_STATE_NAME
        msg.position = np.asarray(
            out.manip_state.arm_state.jnt.position, dtype=np.float64).tolist()
        msg.velocity = np.asarray(
            out.manip_state.arm_state.jnt.velocity, dtype=np.float64).tolist()
        msg.effort = np.asarray(
            out.manip_state.arm_state.jnt.effort, dtype=np.float64).tolist()
        self.__joint_state_pub.publish(msg)

    def pub_clock(self, stamp_ns: int):
        msg = Clock()
        msg.clock = rospy.Time(
            int(stamp_ns // 1_000_000_000),
            int(stamp_ns % 1_000_000_000),
        )
        self.__clock_pub.publish(msg)

    def pub_joy_state(self, out: HexDcTeleopHandleStateStamped):
        msg = HexRosTeleopHandleStateStamped()
        msg.header.stamp = rospy.Time(
            int(out.header.stamp.secs),
            int(out.header.stamp.nsecs),
        )
        msg.header.frame_id = out.header.frame_id
        msg.handle_state.axis_x = out.handle_state.axis_x
        msg.handle_state.axis_y = out.handle_state.axis_y
        msg.handle_state.trigger = out.handle_state.trigger
        msg.handle_state.btn_w = out.handle_state.btn_w
        msg.handle_state.btn_x = out.handle_state.btn_x
        msg.handle_state.btn_y = out.handle_state.btn_y
        msg.handle_state.btn_z = out.handle_state.btn_z
        self.__joy_state_pub.publish(msg)

    ####################
    ### subscribers
    ####################
    def __color_cmd_callback(self, msg: ColorRGBA):
        """Convert ColorRGBA (float 0-1) to int 0-255 arrays and push to deque."""
        r = int(msg.r * 255.0)
        g = int(msg.g * 255.0)
        b = int(msg.b * 255.0)
        self._color_cmd_deque.append({
            "r": [r] * 6,
            "g": [g] * 6,
            "b": [b] * 6,
        })
