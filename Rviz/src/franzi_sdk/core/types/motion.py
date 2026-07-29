# -*- coding: utf-8 -*-
"""
motion — 运动模式、系统状态、急停类型。

对应服务端：SdkMotionMode、SdkSystemState、SdkEmergencyStopType（sdk_api_common.h）。
"""

from __future__ import annotations

from enum import IntEnum


class ArmMotionMode(IntEnum):
    """运动/控制模式（set_motion_mode / get_motion_mode）。"""

    IDLE = 0
    JOG_JOINT = 1
    EXC_JOINT = 2
    JOG_CARTESIAN = 3
    INFERENCE_JOINT = 4
    INFERENCE_CARTESIAN = 5
    JOG_STOP = 6
    EXC_LOAD_IDENT = 7
    ADMITTANCE_JPOS = 8
    USER_POS = 9
    TEACH_JPOS = 10
    JOINT_TELE = 11
    TELEOP_POS = 12
    TORQUE_JOINT = 13
    TORQUE_CARTESIAN = 14
    DRAG_JTORQ = 15
    INFERENCE_TORQUE = 16
    TELEOP_TORQUE = 17
    BACK = 18
    USER_TORQUE = 19
    IMPEDANCE_TRACE = 20


class ArmSystemState(IntEnum):
    """系统运行状态（GetSystemState）。"""

    MOVING = 0
    EMERGENCY_STOP = 1
    IDLE = 2
    NOT_ENABLED = 3


class EmergencyStopType(IntEnum):
    """急停类型（set/get_emergency_stop_type）。"""

    ZERO_TYPE = 0
    ONE_TYPE = 1
    TWO_TYPE = 2
    NONE_TYPE = 3
