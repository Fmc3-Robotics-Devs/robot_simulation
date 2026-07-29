#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GetMotionRobotState 示例：读取 MotionControl 单臂完整机器人状态。"""

from __future__ import annotations

import os
import sys
from time import sleep

_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core.client import ArmClient
from core.types import sdk_status_message
from core import logger

# ------------------------------------------------------------------ 运行参数（按需修改）
HOST = "192.168.23.30"
PORT = 8000


def fmt_vec(name: str, vec: list[float] | None, prec: int = 4) -> str:
    if not vec:
        return f"{name}=[]"
    inner = ", ".join(f"{v:.{prec}f}" for v in vec)
    return f"{name}=[{inner}]"


arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)

try:
    rc, state = arm.GetMotionRobotState(2)
    # logger.info(
    #     "GetMotionRobotState %s rc=%s %s",
    #     label,
    #     rc,
    #     sdk_status_message(rc),
    # )
    logger.info("  %s", fmt_vec("q", state.get("q"))) # 关节位置
    logger.info("  %s", fmt_vec("dq", state.get("dq"))) # 关节速度

except KeyboardInterrupt:
    logger.info("Ctrl+C 中断")
finally:
    arm.disconnect()
    logger.info("已断开连接")
