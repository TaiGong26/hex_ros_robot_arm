#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import numpy as np
import threading

from hex_util_runtime import ns_now

import rclpy
import rclpy.node

from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from hex_ros_msgs.msg import (
    HexRosRoboManipStateStamped,
    HexRosTeleopHandleStateStamped,
)

from hex_util_msg.dataclass.dataclass_base import HexDcBaseTime
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboManipStateStamped,
)
from hex_util_msg.dataclass.dataclass_teleop import (
    HexDcTeleopHandleStateStamped,
)

from .interface_base import HelloInterfaceBase


class DataInterface(HelloInterfaceBase):

    def __init__(self, name: str = "unknown"):
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self._logger = self.__node.get_logger()
        self._shutting_down = False
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

        super().__init__(name)

        ### rate parameters
        self.__node.declare_parameter('ctrl_rate', 500.0)
        self.__node.declare_parameter('rate_state', 100.0)
        self._rate_param["ros"] = self.__node.get_parameter('ctrl_rate').value
        self._rate_param["state"] = self.__node.get_parameter('rate_state').value
        self.__rate = self.__node.create_rate(self._rate_param["ros"])

        ### robot parameters (no grip_type — Hello Y6 has no gripper)
        self.__node.declare_parameter('robot_host', "192.168.1.100")
        self.__node.declare_parameter('robot_port', 8439)
        self.__node.declare_parameter('robot_frame_id', "base_link")
        self.__node.declare_parameter('state_buffer_size', 200)
        self.__node.declare_parameter('sens_ts', True)
        self._robot_param = {
            "host": self.__node.get_parameter('robot_host').value,
            "port": self.__node.get_parameter('robot_port').value,
            "frame_id": self.__node.get_parameter('robot_frame_id').value,
            "state_buffer_size": self.__node.get_parameter('state_buffer_size').value,
            "sens_ts": self.__node.get_parameter('sens_ts').value,
        }

        ### publisher — manip_state
        self.__manip_state_pub = self.__node.create_publisher(
            HexRosRoboManipStateStamped,
            'manip_state',
            10,
        )
        ### publisher — joint_states (for robot_state_publisher / rviz)
        self.__joint_state_pub = self.__node.create_publisher(
            JointState,
            'joint_states',
            10,
        )
        ### publisher — joy_state (Hello grip joy)
        self.__joy_state_pub = self.__node.create_publisher(
            HexRosTeleopHandleStateStamped,
            'joy_state',
            10,
        )

        ### NOTE: No manip_ctrl subscriber — Hello Y6 is read-only.

        ### subscriber — color_cmd (RGB LED control, std_msgs/ColorRGBA)
        self.__color_cmd_sub = self.__node.create_subscription(
            ColorRGBA,
            'color_cmd',
            self.__color_cmd_callback,
            10,
        )
        self.__color_cmd_sub

    def sleep(self):
        self.__rate.sleep()

    ####################
    ### ros infrastructure
    ####################
    def ok(self) -> bool:
        return rclpy.ok()

    def shutdown(self):
        if self._shutting_down:
            return
        self._shutting_down = True
        try:
            self.__node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        self.__spin_thread.join()

    def __spin(self):
        try:
            rclpy.spin(self.__node)
        except rclpy.executors.ExternalShutdownException:
            pass

    ####################
    ### logging
    ####################
    def logd(self, msg, *args, **kwargs):
        self._logger.debug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        self._logger.info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        self._logger.warning(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        self._logger.error(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        self._logger.fatal(msg, *args, **kwargs)

    ####################
    ### time source
    ####################
    def now_ns(self) -> int:
        return ns_now()

    def now_stamp(self) -> HexDcBaseTime:
        now = self.__node.get_clock().now()
        secs, nsecs = now.seconds_nanoseconds()
        return HexDcBaseTime(secs=secs, nsecs=nsecs)

    ####################
    ### publishers
    ####################
    def pub_manip_state(self, out: HexDcRoboManipStateStamped):
        msg = HexRosRoboManipStateStamped()
        # ros time stamp
        now_stamp_dc = self.now_stamp()
        msg.header.stamp = Time(
            sec=int(now_stamp_dc.secs),
            nanosec=int(now_stamp_dc.nsecs),
        )
        msg.header.frame_id = out.header.frame_id

        arm = out.manip_state.arm_state
        hardware_stamp = Time(
            sec=int(out.header.stamp.secs),
            nanosec=int(out.header.stamp.nsecs),
        )
        msg.manip_state.arm_state.jnt.header.stamp = hardware_stamp
        msg.manip_state.arm_state.jnt.header.frame_id = out.header.frame_id
        msg.manip_state.arm_state.jnt.name = self._arm_joint_names
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
        msg.manip_state.grip_state.jnt.header.stamp = hardware_stamp
        msg.manip_state.grip_state.jnt.header.frame_id = out.header.frame_id
        msg.manip_state.grip_state.jnt.name = self._arm_joint_names
        msg.manip_state.grip_state.jnt.position = []
        msg.manip_state.grip_state.jnt.velocity = []
        msg.manip_state.grip_state.jnt.effort = []

        self.__manip_state_pub.publish(msg)

    def pub_joint_state(self, out: HexDcRoboManipStateStamped):
        msg = JointState()
        now_stamp_dc = self.now_stamp()
        msg.header.stamp = Time(
            sec=int(now_stamp_dc.secs),
            nanosec=int(now_stamp_dc.nsecs),
        )
        msg.header.frame_id = out.header.frame_id
        msg.name = self._arm_joint_names
        msg.position = np.asarray(
            out.manip_state.arm_state.jnt.position, dtype=np.float64).tolist()
        msg.velocity = np.asarray(
            out.manip_state.arm_state.jnt.velocity, dtype=np.float64).tolist()
        msg.effort = np.asarray(
            out.manip_state.arm_state.jnt.effort, dtype=np.float64).tolist()
        self.__joint_state_pub.publish(msg)

    def pub_joy_state(self, out: HexDcTeleopHandleStateStamped):
        msg = HexRosTeleopHandleStateStamped()
        now_stamp_dc = self.now_stamp()
        msg.header.stamp = Time(
            sec=int(now_stamp_dc.secs),
            nanosec=int(now_stamp_dc.nsecs),
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
