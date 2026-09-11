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
from utility import DataInterface

from hex_driver_robot import HexRobotFireflyY6, HexRobotFireflyY6Params

from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboArmCtrlMode,
    HexDcRoboArmStateStamped,
    HexDcRoboGripCtrlMode,
    HexDcRoboGripStateStamped,
    HexDcRoboGripState,
    HexDcRoboManipCtrlStamped,
    HexDcRoboManipState,
    HexDcRoboManipStateStamped,
)
from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseHeader,
    HexDcBaseJntState,
)

TYPE = {
    "H1": 27,
    "H2_40": 39,
}


class RobotFireflyY6:

    def __init__(self):
        ### utility
        self.__data_interface = DataInterface("hex_ros_robot_firefly_y6")

        ### parameters
        rate_param = self.__data_interface.get_rate_param()
        robot_param = self.__data_interface.get_robot_param()
        self.__data_interface.logi(f"ctrl_rate: {rate_param['ros']} hz")
        self.__data_interface.logi(f"rate_state: {rate_param['state']} hz")
        self.__data_interface.logi(f"robot_host: {robot_param['host']}")
        self.__data_interface.logi(f"robot_port: {robot_param['port']}")
        self.__data_interface.logi(f"robot_frame_id: {robot_param['frame_id']}")
        self.__data_interface.logi(f"robot_grip_type: {robot_param['grip_type']}")
        self.__data_interface.logi(f"robot_enable_kcp: {robot_param['enable_kcp']}")
        self.__data_interface.logi(f"state_buffer_size: {robot_param['state_buffer_size']}")
        self.__data_interface.logi(f"sens_ts: {robot_param['sens_ts']}")

        ### robot driver
        self.__robot = HexRobotFireflyY6(HexRobotFireflyY6Params(
            host=robot_param["host"],
            port=robot_param["port"],
            ctrl_rate=rate_param["ros"],
            state_buffer_size=robot_param["state_buffer_size"],
            sens_ts=robot_param["sens_ts"],
            grip_type=robot_param["grip_type"],
            enable_kcp=robot_param["enable_kcp"],
            robot_type=TYPE.get(robot_param["variant"], 27)
        ))
        self.__robot.start()

        self.__data_interface.set_joint_names(self.__robot.get_dofs())

        ### derived
        self.__state_decim = max(
            1,
            int(round(rate_param["ros"] / rate_param["state"])),
        )
        self.__robot_frame_id = robot_param["frame_id"]

    def __apply_manip_ctrl(self, ctrl: HexDcRoboManipCtrlStamped):
        arm_ctrl = ctrl.manip_ctrl.arm_ctrl
        grip_ctrl = ctrl.manip_ctrl.grip_ctrl

        # --- Arm ---
        if arm_ctrl.ctrl_mode == HexDcRoboArmCtrlMode.MIT:
            cmd = {
                "ts_ns": self.__data_interface.now_ns(),
                "jnt_pos": arm_ctrl.jnt.pos.copy(),
                "jnt_vel": arm_ctrl.jnt.vel.copy(),
                "mit_tau": arm_ctrl.jnt.eff.copy(),
                "mit_kp": arm_ctrl.jnt.kp.copy(),
                "mit_kd": arm_ctrl.jnt.kd.copy(),
            }
            if arm_ctrl.grav.x != 0.0 or arm_ctrl.grav.y != 0.0 or arm_ctrl.grav.z != 0.0:
                cmd["grav"] = arm_ctrl.grav
            self.__robot.set_arm_mit_cmd(cmd)

        elif arm_ctrl.ctrl_mode == HexDcRoboArmCtrlMode.JNT:
            cmd = {
                "ts_ns": self.__data_interface.now_ns(),
                "jnt_pos": arm_ctrl.jnt.pos.copy(),
            }
            if len(arm_ctrl.jnt.eff) > 0:
                cmd["jnt_eff"] = arm_ctrl.jnt.eff.copy()
            if len(arm_ctrl.jnt.lim_vel) > 0:
                cmd["lim_vel"] = float(arm_ctrl.jnt.lim_vel[0])
            if len(arm_ctrl.jnt.lim_acc) > 0:
                cmd["lim_acc"] = float(arm_ctrl.jnt.lim_acc[0])
            if arm_ctrl.grav.x != 0.0 or arm_ctrl.grav.y != 0.0 or arm_ctrl.grav.z != 0.0:
                cmd["grav"] = arm_ctrl.grav
            self.__robot.set_arm_pos_cmd(cmd)

        elif arm_ctrl.ctrl_mode == HexDcRoboArmCtrlMode.EE:
            cmd = {
                "ts_ns": self.__data_interface.now_ns(),
                "pose_pos": [
                    arm_ctrl.pose.position.x,
                    arm_ctrl.pose.position.y,
                    arm_ctrl.pose.position.z,
                ],
                "pose_quat": [
                    arm_ctrl.pose.orientation.w,
                    arm_ctrl.pose.orientation.x,
                    arm_ctrl.pose.orientation.y,
                    arm_ctrl.pose.orientation.z,
                ],
            }
            if len(arm_ctrl.jnt.eff) > 0:
                cmd["jnt_eff"] = arm_ctrl.jnt.eff.copy()
            if len(arm_ctrl.jnt.lim_vel) > 0:
                cmd["lim_vel"] = float(arm_ctrl.jnt.lim_vel[0])
            if len(arm_ctrl.jnt.lim_acc) > 0:
                cmd["lim_acc"] = float(arm_ctrl.jnt.lim_acc[0])
            if arm_ctrl.grav.x != 0.0 or arm_ctrl.grav.y != 0.0 or arm_ctrl.grav.z != 0.0:
                cmd["grav"] = arm_ctrl.grav
            self.__robot.set_arm_pose_cmd(cmd)

        # arm NONE: no-op

        # --- Grip ---
        if grip_ctrl.ctrl_mode == HexDcRoboGripCtrlMode.MIT:
            self.__robot.set_grip_mit_cmd({
                "ts_ns": self.__data_interface.now_ns(),
                "jnt_pos": grip_ctrl.jnt.pos.copy(),
                "jnt_vel": grip_ctrl.jnt.vel.copy(),
                "mit_tau": grip_ctrl.jnt.eff.copy(),
                "mit_kp": grip_ctrl.jnt.kp.copy(),
                "mit_kd": grip_ctrl.jnt.kd.copy(),
            })

        elif grip_ctrl.ctrl_mode == HexDcRoboGripCtrlMode.JNT:
            cmd = {
                "ts_ns": self.__data_interface.now_ns(),
                "jnt_pos": grip_ctrl.jnt.pos.copy(),
            }
            if len(grip_ctrl.jnt.eff) > 0:
                cmd["jnt_eff"] = float(grip_ctrl.jnt.eff[0])
            if len(grip_ctrl.jnt.lim_vel) > 0:
                cmd["lim_vel"] = float(grip_ctrl.jnt.lim_vel[0])
            self.__robot.set_grip_pos_cmd(cmd)

        elif grip_ctrl.ctrl_mode == HexDcRoboGripCtrlMode.TAU:
            cmd = {
                "ts_ns": self.__data_interface.now_ns(),
                "jnt_eff": grip_ctrl.jnt.eff.copy(),
            }
            if len(grip_ctrl.jnt.lim_vel) > 0:
                cmd["lim_vel"] = float(grip_ctrl.jnt.lim_vel[0])
            self.__robot.set_grip_force_cmd(cmd)

        # grip NONE: no-op

    def __build_manip_state(
        self,
        arm_state: Optional[HexDcRoboArmStateStamped],
        grip_state: Optional[HexDcRoboGripStateStamped],
    ) -> Optional[HexDcRoboManipStateStamped]:
        if arm_state is None:
            return None

        if grip_state is not None:
            grip = grip_state.grip_state
        else:
            grip = HexDcRoboGripState(
                jnt=HexDcBaseJntState(),
            )

        return HexDcRoboManipStateStamped(
            header=HexDcBaseHeader(
                stamp=arm_state.header.stamp,
                frame_id=self.__robot_frame_id,
            ),
            manip_state=HexDcRoboManipState(
                arm_state=arm_state.arm_state,
                grip_state=grip,
            ),
        )

    def run(self):
        state_count = 0
        while self.__data_interface.ok() and self.__robot.is_working():
            # 1. drain to the latest control frame
            ctrl = self.__data_interface.get_manip_ctrl(latest=True)
            if ctrl is not None:
                self.__apply_manip_ctrl(ctrl)

            # 3. publish robot state at the requested rate
            state_count += 1
            if state_count >= self.__state_decim:
                state_count = 0

                arm_state = self.__robot.get_arm_state()
                grip_state = self.__robot.get_grip_state()

                manip_state = self.__build_manip_state(arm_state, grip_state)
                if manip_state is not None:
                    self.__data_interface.pub_manip_state(manip_state)
                    self.__data_interface.pub_joint_state(manip_state)

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
    robot_firefly_y6 = RobotFireflyY6()
    try:
        robot_firefly_y6.run()
    except KeyboardInterrupt:
        pass
    finally:
        robot_firefly_y6.shutdown()


if __name__ == '__main__':
    main()
