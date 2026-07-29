# -*- coding: utf-8 -*-
"""
shared — 跨域 RPC 枚举与请求构造（对应 service/sdk_service_core.h 等共享段）
"""

from __future__ import annotations

from enum import IntEnum
from typing import Any, Dict, Union

from core.errors import ArmSdkError, sdk_status_message, sdk_status_name
from core.types.mechunit import (  # noqa: F401
    MechUnitLifecycleCmd,
    MechUnitStateType,
    MechUnitType,
    format_mechunit_error_code,
    mech_unit_state_name,
    mech_unit_type_name,
)

RPC_PREFIX = "arm_sdk."


class SdkStatus(IntEnum):
    OK = 0
    SYNTAX_ERROR = -1
    PARAM_COUNT_MISMATCH = -2
    PARAM_INVALID = -3
    UNSUPPORTED = -4
    NOT_READY = -5


class ArmSide(IntEnum):
    LEFT = 0
    RIGHT = 1
    BIO = 2


ARM_SIDE_DUAL = -1
SideLike = Union[ArmSide, int]


def side_value(side: SideLike) -> int:
    return int(side) if isinstance(side, ArmSide) else int(side)


class ArmMotionMode(IntEnum):
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
    MOVING = 0
    EMERGENCY_STOP = 1
    IDLE = 2
    NOT_ENABLED = 3


class EmergencyStopType(IntEnum):
    ZERO_TYPE = 0
    ONE_TYPE = 1
    TWO_TYPE = 2
    NONE_TYPE = 3


def side_req(s: SideLike) -> Dict[str, int]:
    return {"side": side_value(s)}


def side_bool_req(s: SideLike, value: bool) -> Dict[str, Any]:
    return {"side": side_value(s), "value": bool(value)}


def robot_type_req(robot_type: int, lifecycle_cmd: int) -> Dict[str, int]:
    return {"robot_type": int(robot_type), "lifecycle_cmd": int(lifecycle_cmd)}


def component_type_req(component_type: int) -> Dict[str, int]:
    return {"component_type": int(component_type)}


def component_lifecycle_req(component_type: int, lifecycle_cmd: int) -> Dict[str, int]:
    return {"component_type": int(component_type), "lifecycle_cmd": int(lifecycle_cmd)}


def side_mode_req(s: SideLike, mode: Union[ArmMotionMode, int]) -> Dict[str, int]:
    m = int(mode.value) if isinstance(mode, ArmMotionMode) else int(mode)
    return {"side": side_value(s), "mode": m}


def side_estop_req(s: SideLike, estop: Union[EmergencyStopType, int]) -> Dict[str, int]:
    e = int(estop.value) if isinstance(estop, EmergencyStopType) else int(estop)
    return {"side": side_value(s), "estop_type": e}
