# -*- coding: utf-8 -*-
"""
side — 臂选择与 side 相关常量。

对应服务端：SdkSideReq（sdk_api_common.h）。
"""

from __future__ import annotations

from enum import IntEnum
from typing import Union


class ArmSide(IntEnum):
    """机械臂侧别（SdkSideReq.side）。"""

    LEFT = 0
    RIGHT = 1
    BIO = 2


ARM_SIDE_DUAL = -1
"""双臂合并 RPC 使用的 side（side < 0，与 C++ 约定一致）。"""

SideLike = Union[ArmSide, int]
"""side 参数可传 ArmSide 或 int。"""


def side_value(side: SideLike) -> int:
    """
    转为 RPC 用 side 整型。

    :param side: ArmSide 或 int
    :return: side 字段值
    """
    return int(side) if isinstance(side, ArmSide) else int(side)
