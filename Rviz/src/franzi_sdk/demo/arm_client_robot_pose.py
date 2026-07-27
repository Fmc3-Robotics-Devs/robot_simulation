#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GetRobotPose 示例：读取左右臂末端 6D 位姿。"""

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


def fmt_pose(pose: list[float]) -> str:
    if len(pose) < 6:
        return str(pose)
    return (
        f"xyz=[{pose[0]:.4f}, {pose[1]:.4f}, {pose[2]:.4f}] "
        f"euler=[{pose[3]:.4f}, {pose[4]:.4f}, {pose[5]:.4f}]"
    )


arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)

try:
    logger.info("SDK version=%s IsReady=%s", arm.GetVersion(), arm.IsReady())

    for side, label in ((0, "left"), (1, "right")):
        rc, pose = arm.GetRobotPose(side)
        logger.info(
            "GetRobotPose %s rc=%s %s pose=%s",
            label,
            rc,
            sdk_status_message(rc),
            fmt_pose(pose),
        )

except KeyboardInterrupt:
    logger.info("Ctrl+C 中断")
finally:
    arm.disconnect()
    logger.info("已断开连接")
