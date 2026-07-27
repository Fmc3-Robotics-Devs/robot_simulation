# -*- coding: utf-8 -*-
"""MechUnit 整机状态枚举（对应 mechunit_types.h / SdkRobotState RPC）。"""

from __future__ import annotations

from enum import IntEnum


class MechUnitType(IntEnum):
    LEFTARM = 0
    RIGHTARM = 10
    WAIST = 20
    HEAD = 30
    LEFTHAND = 40
    RIGHTHAND = 50
    UNKNOWN = 255


class MechUnitStateType(IntEnum):
    FAULT = 0
    RESETTING = 10
    DISABLED = 20
    DISABLING = 25
    ENABLING = 30
    ENABLED = 40
    RUNNING = 50


class MechUnitLifecycleCmd(IntEnum):
    """SystemModule MechUnitLifecycleCmdCode（SetMechUnitLifecycle）。"""

    ENABLE = 1
    DISABLE = 2
    RESET = 3


_MECH_UNIT_TYPE_ZH = {
    MechUnitType.LEFTARM: "左臂",
    MechUnitType.RIGHTARM: "右臂",
    MechUnitType.WAIST: "腰",
    MechUnitType.HEAD: "头",
    MechUnitType.LEFTHAND: "左手",
    MechUnitType.RIGHTHAND: "右手",
}

_MECH_STATE_ZH = {
    MechUnitStateType.FAULT: "故障/未激活",
    MechUnitStateType.RESETTING: "复位中",
    MechUnitStateType.DISABLED: "未使能",
    MechUnitStateType.DISABLING: "下使能中",
    MechUnitStateType.ENABLING: "上使能中",
    MechUnitStateType.ENABLED: "已使能",
    MechUnitStateType.RUNNING: "运动中",
}


def mech_unit_type_name(value: int) -> str:
    try:
        t = MechUnitType(int(value))
        zh = _MECH_UNIT_TYPE_ZH.get(t, "")
        return f"{t.name}({int(t)}) {zh}".strip()
    except ValueError:
        return f"UNKNOWN({int(value)})"


def mech_unit_state_name(value: int) -> str:
    try:
        s = MechUnitStateType(int(value))
        zh = _MECH_STATE_ZH.get(s, "")
        return f"{s.name}({int(s)}) {zh}".strip()
    except ValueError:
        return f"UNKNOWN({int(value)})"


def format_mechunit_error_code(value: int) -> str:
    """将关节/单元 fault error_code 格式化为 32 位十六进制（如 ``0x00001234``）。"""
    v = int(value) & 0xFFFFFFFF
    return f"0x{v:08X}"
