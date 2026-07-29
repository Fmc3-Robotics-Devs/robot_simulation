# -*- coding: utf-8 -*-
"""
requests — msgpack 请求体构造辅助函数。

将 types.side / types.motion 组合为 RPC 请求 dict。
"""

from __future__ import annotations

from typing import Any, Dict, Union

import core.types.motion as motion
from core import types as side


def side_req(s: side.SideLike) -> Dict[str, int]:
    """
    构造仅含 side 的请求体。

    :param s: 臂选择
    :return: ``{"side": int}``
    """
    return {"side": side.side_value(s)}


def side_bool_req(s: side.SideLike, value: bool) -> Dict[str, Any]:
    """
    构造 {side, value}，如 set_enable_state。

    :param s: 臂选择
    :param value: 布尔字段
    :return: 请求 dict
    """
    return {"side": side.side_value(s), "value": bool(value)}


def robot_type_req(robot_type: int, lifecycle_cmd: int) -> Dict[str, int]:
    """
    构造 {robot_type, lifecycle_cmd}，如 set_enable_state（MechUnit 生命周期指令）。

    :param robot_type: MechUnitType，如 0=LEFTARM, 10=RIGHTARM
    :param lifecycle_cmd: 0=None, 1=Enable, 2=Disable, 3=Reset
    :return: 请求 dict
    """
    return {"robot_type": int(robot_type), "lifecycle_cmd": int(lifecycle_cmd)}


def side_mode_req(
    s: side.SideLike, mode: Union[motion.ArmMotionMode, int]
) -> Dict[str, int]:
    """
    构造 {side, mode}，如 set_motion_mode。

    :param s: 臂选择
    :param mode: ArmMotionMode 或 int
    :return: 请求 dict
    """
    m = int(mode.value) if isinstance(mode, motion.ArmMotionMode) else int(mode)
    return {"side": side.side_value(s), "mode": m}


def side_estop_req(
    s: side.SideLike, estop: Union[motion.EmergencyStopType, int]
) -> Dict[str, int]:
    """
    构造 {side, estop_type}，如 set_emergency_stop_type。

    :param s: 臂选择
    :param estop: EmergencyStopType 或 int
    :return: 请求 dict
    """
    e = int(estop.value) if isinstance(estop, motion.EmergencyStopType) else int(estop)
    return {"side": side.side_value(s), "estop_type": e}
