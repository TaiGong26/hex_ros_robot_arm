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
from geometry_msgs.msg import Pose
from hex_ros_msgs.msg import (
    HexRosJnt,
    HexRosRoboManipStateStamped,
    HexRosRoboManipCtrlStamped,
)

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseHeader,
    HexDcBaseTime,
    HexDcBaseVector3,
    HexDcBaseQuaternion,
    HexDcBasePose,
    HexDcBaseJntFull,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboArmCtrl,
    HexDcRoboArmCtrlMode,
    HexDcRoboGripCtrl,
    HexDcRoboGripCtrlMode,
    HexDcRoboManipCtrl,
    HexDcRoboManipCtrlStamped,
    HexDcRoboManipStateStamped,
)

from .interface_base import ArmInterfaceBase
from .interface_base import JOINT_STATE_NAME


class DataInterface(ArmInterfaceBase):

    def __init__(self, name: str = "unknown"):
        rospy.init_node(name, anonymous=True)
        super().__init__(name)

        ### rate parameters
        self._rate_param["ros"] = rospy.get_param('~ctrl_rate', 500.0)
        self._rate_param["state"] = rospy.get_param('~rate_state', 100.0)
        self.__rate = rospy.Rate(self._rate_param["ros"])

        ### robot parameters
        self._robot_param = {
            "host": rospy.get_param('~robot_host', "192.168.1.100"),
            "port": rospy.get_param('~robot_port', 8439),
            "frame_id": rospy.get_param('~robot_frame_id', "base_link"),
            "grip_type": rospy.get_param('~robot_grip_type', "gp80"),
            "enable_kcp": rospy.get_param('~robot_enable_kcp', True),
            "state_buffer_size": rospy.get_param('~state_buffer_size', 200),
            "sens_ts": rospy.get_param('~sens_ts', False),
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

        ### subscriber — manip_ctrl
        self.__manip_ctrl_sub = rospy.Subscriber(
            'manip_ctrl',
            HexRosRoboManipCtrlStamped,
            self.__manip_ctrl_callback,
        )
        self.__manip_ctrl_sub

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

        grip = out.manip_state.grip_state
        msg.manip_state.grip_state.jnt.position = \
            np.asarray(grip.jnt.position, dtype=np.float64).tolist()
        msg.manip_state.grip_state.jnt.velocity = \
            np.asarray(grip.jnt.velocity, dtype=np.float64).tolist()
        msg.manip_state.grip_state.jnt.effort = \
            np.asarray(grip.jnt.effort, dtype=np.float64).tolist()

        self.__manip_state_pub.publish(msg)

    def pub_joint_state(self, out: HexDcRoboManipStateStamped):
        msg = JointState()
        msg.header.stamp = rospy.Time(
            int(out.header.stamp.secs),
            int(out.header.stamp.nsecs),
        )
        msg.header.frame_id = out.header.frame_id
        msg.name = JOINT_STATE_NAME
        msg.position = np.concatenate([
            np.asarray(out.manip_state.arm_state.jnt.position,
                       dtype=np.float64),
            np.asarray(out.manip_state.grip_state.jnt.position,
                       dtype=np.float64),
        ]).tolist()
        msg.velocity = np.concatenate([
            np.asarray(out.manip_state.arm_state.jnt.velocity,
                       dtype=np.float64),
            np.asarray(out.manip_state.grip_state.jnt.velocity,
                       dtype=np.float64),
        ]).tolist()
        msg.effort = np.concatenate([
            np.asarray(out.manip_state.arm_state.jnt.effort,
                       dtype=np.float64),
            np.asarray(out.manip_state.grip_state.jnt.effort,
                       dtype=np.float64),
        ]).tolist()
        self.__joint_state_pub.publish(msg)

    def pub_clock(self, stamp_ns: int):
        msg = Clock()
        msg.clock = rospy.Time(
            int(stamp_ns // 1_000_000_000),
            int(stamp_ns % 1_000_000_000),
        )
        self.__clock_pub.publish(msg)

    def __manip_ctrl_callback(self, msg: HexRosRoboManipCtrlStamped):
        self._manip_ctrl_deque.append(self.__manip_ctrl_msg_to_dc(msg))

    @staticmethod
    def __manip_ctrl_msg_to_dc(
            msg: HexRosRoboManipCtrlStamped) -> HexDcRoboManipCtrlStamped:
        header = HexDcBaseHeader(
            stamp=HexDcBaseTime(
                secs=int(msg.header.stamp.secs),
                nsecs=int(msg.header.stamp.nsecs),
            ),
            frame_id=msg.header.frame_id,
        )

        arm_msg = msg.manip_ctrl.arm_ctrl
        arm_ctrl = HexDcRoboArmCtrl(
            ctrl_mode=HexDcRoboArmCtrlMode(int(arm_msg.ctrl_mode)),
            grav=HexDcBaseVector3(
                x=arm_msg.grav.x,
                y=arm_msg.grav.y,
                z=arm_msg.grav.z,
            ),
            jnt=DataInterface.__jnt_to_dc(arm_msg.jnt),
            pose=DataInterface.__pose_to_dc(arm_msg.pose),
        )

        grip_msg = msg.manip_ctrl.grip_ctrl
        grip_ctrl = HexDcRoboGripCtrl(
            ctrl_mode=HexDcRoboGripCtrlMode(int(grip_msg.ctrl_mode)),
            jnt=DataInterface.__jnt_to_dc(grip_msg.jnt),
        )

        return HexDcRoboManipCtrlStamped(
            header=header,
            manip_ctrl=HexDcRoboManipCtrl(
                arm_ctrl=arm_ctrl,
                grip_ctrl=grip_ctrl,
            ),
        )

    @staticmethod
    def __jnt_to_dc(jnt: HexRosJnt) -> HexDcBaseJntFull:
        return HexDcBaseJntFull(
            pos=np.asarray(jnt.pos, dtype=np.float64),
            vel=np.asarray(jnt.vel, dtype=np.float64),
            eff=np.asarray(jnt.eff, dtype=np.float64),
            kp=np.asarray(jnt.kp, dtype=np.float64),
            kd=np.asarray(jnt.kd, dtype=np.float64),
            lim_vel=np.asarray(jnt.lim_vel, dtype=np.float64),
            lim_acc=np.asarray(jnt.lim_acc, dtype=np.float64),
        )

    @staticmethod
    def __pose_to_dc(pose: Pose) -> HexDcBasePose:
        return HexDcBasePose(
            position=HexDcBaseVector3(
                x=pose.position.x,
                y=pose.position.y,
                z=pose.position.z,
            ),
            orientation=HexDcBaseQuaternion(
                x=pose.orientation.x,
                y=pose.orientation.y,
                z=pose.orientation.z,
                w=pose.orientation.w,
            ),
        )
