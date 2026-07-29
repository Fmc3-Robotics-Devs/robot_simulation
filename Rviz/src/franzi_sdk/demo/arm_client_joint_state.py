#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GetJointState 示例：读取左右臂关节位置。"""

from __future__ import annotations

import os
import sys

_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core.client import ArmClient
from core.types import sdk_status_message
from core import logger

# ------------------------------------------------------------------ 运行参数（按需修改）
HOST = "192.168.23.30"
PORT = 8000


def fmt_joints(values: list[float], unit: str) -> str:
    if not values:
        return "[]"
    return "[" + ", ".join(f"{v:.4f}" for v in values) + f"] {unit}"


arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)

try:
    logger.info("SDK version=%s IsReady=%s", arm.GetVersion(), arm.IsReady())

    for side, label in ((0, "left"), (1, "right")):
        rc, positions = arm.GetJointState(side)
        logger.info(
            "GetJointState %s rc=%s %s position=%s",
            label,
            rc,
            sdk_status_message(rc),
            fmt_joints(positions, "rad"),
        )

except KeyboardInterrupt:
    logger.info("Ctrl+C 中断")
finally:
    arm.disconnect()
    logger.info("已断开连接")
