#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import os
import sys
from typing import Optional

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from hello_utils import DataInterface

from hex_driver_robot import HexRobotHelloY6, HexRobotHelloY6Params

from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboArmStateStamped,
    HexDcRoboGripState,
    HexDcRoboManipState,
    HexDcRoboManipStateStamped,
)
from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseHeader,
    HexDcBaseTime,
    HexDcBaseJntState,
)
from hex_util_msg.dataclass.dataclass_teleop import (
    HexDcTeleopHandleState,
    HexDcTeleopHandleStateStamped,
)


class RobotHelloY6:

    def __init__(self):
        ### utility
        self.__data_interface = DataInterface("hex_ros_robot_hello_y6")

        ### parameters
        rate_param = self.__data_interface.get_rate_param()
        robot_param = self.__data_interface.get_robot_param()
        self.__data_interface.logi(f"ctrl_rate: {rate_param['ros']} hz")
        self.__data_interface.logi(f"rate_state: {rate_param['state']} hz")
        self.__data_interface.logi(f"robot_host: {robot_param['host']}")
        self.__data_interface.logi(f"robot_port: {robot_param['port']}")
        self.__data_interface.logi(f"robot_frame_id: {robot_param['frame_id']}")
        self.__data_interface.logi(f"state_buffer_size: {robot_param['state_buffer_size']}")
        self.__data_interface.logi(f"sens_ts: {robot_param['sens_ts']}")

        ### robot driver (Hello Y6 — read-only arm + joystick)
        self.__robot = HexRobotHelloY6(HexRobotHelloY6Params(
            host=robot_param["host"],
            port=robot_param["port"],
            ctrl_rate=rate_param["ros"],
            state_buffer_size=robot_param["state_buffer_size"],
            sens_ts=robot_param["sens_ts"],
        ))
        self.__robot.start()

        self.__data_interface.set_joint_names(self.__robot.get_dofs())

        ### derived
        self.__state_decim = max(
            1,
            int(round(rate_param["ros"] / rate_param["state"])),
        )
        self.__robot_frame_id = robot_param["frame_id"]

    def __build_manip_state(
        self,
        arm_state: Optional[HexDcRoboArmStateStamped],
    ) -> Optional[HexDcRoboManipStateStamped]:
        if arm_state is None:
            return None

        # Hello Y6 has no gripper — publish empty grip state
        return HexDcRoboManipStateStamped(
            header=HexDcBaseHeader(
                stamp=arm_state.header.stamp,
                frame_id=self.__robot_frame_id,
            ),
            manip_state=HexDcRoboManipState(
                arm_state=arm_state.arm_state,
                grip_state=HexDcRoboGripState(
                    jnt=HexDcBaseJntState(),
                ),
            ),
        )

    def __build_joy_state(self, grip_joy):
        if grip_joy is None:
            return None
        return HexDcTeleopHandleStateStamped(
            header=HexDcBaseHeader(
                stamp=HexDcBaseTime(
                    secs=grip_joy.ts_ns // 1_000_000_000,
                    nsecs=grip_joy.ts_ns % 1_000_000_000,
                ),
                frame_id=self.__robot_frame_id,
            ),
            handle_state=HexDcTeleopHandleState(
                axis_x=grip_joy.joystick_x,
                axis_y=grip_joy.joystick_y,
                trigger=grip_joy.trigger,
                btn_w=grip_joy.btn_w,
                btn_x=grip_joy.btn_x,
                btn_y=grip_joy.btn_y,
                btn_z=grip_joy.btn_z,
            ),
        )

    def run(self):
        state_count = 0
        while self.__data_interface.ok() and self.__robot.is_working():
            # 1. drain color command (RGB LED)
            color_cmd = self.__data_interface.get_color_cmd(latest=True)
            if color_cmd is not None:
                self.__robot.set_rgb_cmd(color_cmd)

            # 3. publish robot state at the requested rate
            state_count += 1
            if state_count >= self.__state_decim:
                state_count = 0
                
                arm_state = self.__robot.get_arm_state()
                joy_state = self.__build_joy_state(self.__robot.get_grip_joy())
                
                manip_state = self.__build_manip_state(arm_state)
                if manip_state is not None:
                    self.__data_interface.pub_manip_state(manip_state)
                    self.__data_interface.pub_joint_state(manip_state)
                if joy_state is not None:
                    self.__data_interface.pub_joy_state(joy_state)

            self.__data_interface.sleep()

    def shutdown(self):
        try:
            self.__robot.stop()
        except Exception:
            pass
        try:
            self.__data_interface.shutdown()
        except Exception:
            pass


def main():
    robot_hello_y6 = RobotHelloY6()
    try:
        robot_hello_y6.run()
    except KeyboardInterrupt:
        pass
    finally:
        robot_hello_y6.shutdown()


if __name__ == '__main__':
    main()
