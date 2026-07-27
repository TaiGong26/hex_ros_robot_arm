#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

from collections import deque
from typing import Any, Optional
from abc import ABC, abstractmethod

from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboManipCtrlStamped,
    HexDcRoboManipStateStamped,
)

JOINT_STATE_NAME = [f"joint_{i}" for i in range(1, 7)] + ["grip_joint_1"]


class ArmInterfaceBase(ABC):

    def __init__(self, name: str = "unknown"):
        self._name = name

        ### ros parameters
        self._rate_param = {}
        self._robot_param = {}

        ### rx msg queues
        self._manip_ctrl_deque = deque(maxlen=100)

    ####################
    ### ros infrastructure
    ####################
    @abstractmethod
    def ok(self) -> bool:
        raise NotImplementedError("ArmInterfaceBase.ok")

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError("ArmInterfaceBase.shutdown")

    @abstractmethod
    def sleep(self):
        raise NotImplementedError("ArmInterfaceBase.sleep")

    @abstractmethod
    def now_ns(self) -> int:
        raise NotImplementedError("ArmInterfaceBase.now_ns")

    ####################
    ### logging
    ####################
    @abstractmethod
    def logd(self, msg, *args, **kwargs):
        raise NotImplementedError("ArmInterfaceBase.logd")

    @abstractmethod
    def logi(self, msg, *args, **kwargs):
        raise NotImplementedError("ArmInterfaceBase.logi")

    @abstractmethod
    def logw(self, msg, *args, **kwargs):
        raise NotImplementedError("ArmInterfaceBase.logw")

    @abstractmethod
    def loge(self, msg, *args, **kwargs):
        raise NotImplementedError("ArmInterfaceBase.loge")

    @abstractmethod
    def logf(self, msg, *args, **kwargs):
        raise NotImplementedError("ArmInterfaceBase.logf")

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
        raise NotImplementedError("ArmInterfaceBase.pub_manip_state")

    @abstractmethod
    def pub_joint_state(self, out: HexDcRoboManipStateStamped):
        raise NotImplementedError("ArmInterfaceBase.pub_joint_state")

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

    # manip ctrl
    def get_manip_ctrl(
        self,
        latest: bool = False,
    ) -> Optional[HexDcRoboManipCtrlStamped]:
        return self.deque_helper(self._manip_ctrl_deque, latest)
