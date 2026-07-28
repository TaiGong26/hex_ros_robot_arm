#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

from collections import deque
from typing import Any, Dict, List, Optional
from abc import ABC, abstractmethod

from hex_util_msg.dataclass.dataclass_base import HexDcBaseTime
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboManipStateStamped,
)
from hex_util_msg.dataclass.dataclass_teleop import (
    HexDcTeleopHandleStateStamped,
)


# 6 arm joints (Hello Y6 has no gripper)
JOINT_STATE_NAME = [
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6",
]


class HelloInterfaceBase(ABC):

    def __init__(self, name: str = "unknown"):
        self._name = name

        ### ros parameters
        self._rate_param = {}
        self._robot_param = {}

        ### rx msg queues
        self._color_cmd_deque = deque(maxlen=10)

    ####################
    ### ros infrastructure
    ####################
    @abstractmethod
    def ok(self) -> bool:
        raise NotImplementedError("HelloInterfaceBase.ok")

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError("HelloInterfaceBase.shutdown")

    @abstractmethod
    def sleep(self):
        raise NotImplementedError("HelloInterfaceBase.sleep")

    @abstractmethod
    def now_ns(self) -> int:
        raise NotImplementedError("HelloInterfaceBase.now_ns")

    @abstractmethod
    def now_stamp(self) -> HexDcBaseTime:
        raise NotImplementedError("HelloInterfaceBase.now_stamp")

    ####################
    ### logging
    ####################
    @abstractmethod
    def logd(self, msg, *args, **kwargs):
        raise NotImplementedError("HelloInterfaceBase.logd")

    @abstractmethod
    def logi(self, msg, *args, **kwargs):
        raise NotImplementedError("HelloInterfaceBase.logi")

    @abstractmethod
    def logw(self, msg, *args, **kwargs):
        raise NotImplementedError("HelloInterfaceBase.logw")

    @abstractmethod
    def loge(self, msg, *args, **kwargs):
        raise NotImplementedError("HelloInterfaceBase.loge")

    @abstractmethod
    def logf(self, msg, *args, **kwargs):
        raise NotImplementedError("HelloInterfaceBase.logf")

    ####################
    ### parameters
    ####################
    def get_rate_param(self) -> dict:
        return self._rate_param

    def get_robot_param(self) -> dict:
        return self._robot_param

    ####################
    ### publishers
    ####################
    @abstractmethod
    def pub_manip_state(self, out: HexDcRoboManipStateStamped):
        raise NotImplementedError("HelloInterfaceBase.pub_manip_state")

    @abstractmethod
    def pub_joint_state(self, out: HexDcRoboManipStateStamped):
        raise NotImplementedError("HelloInterfaceBase.pub_joint_state")

    @abstractmethod
    def pub_joy_state(self, out: HexDcTeleopHandleStateStamped):
        raise NotImplementedError("HelloInterfaceBase.pub_joy_state")

    ####################
    ### subscribers
    ####################
    @staticmethod
    def deque_helper(dq: deque, latest: bool = False) -> Optional[Any]:
        if not latest:
            if dq:
                return dq.popleft()
            else:
                return None
        else:
            if dq:
                ret = dq[-1]
                dq.clear()
                return ret
            else:
                return None

    # color command (RGB LED — Hello Y6)
    def get_color_cmd(self, latest: bool = False) -> Optional[Dict[str, List[int]]]:
        return self.deque_helper(self._color_cmd_deque, latest)
