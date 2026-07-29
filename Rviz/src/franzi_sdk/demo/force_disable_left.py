#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ---------------------------------------------------------------------------
# Python SDK 力控测试用例 1：机器人使能状态设置测试
# ---------------------------------------------------------------------------
# 测试目的：
# 1. 验证 SetEnableState 接口能否正常下发。
#
# 测试力控前请确认：
# 1. 机器人已经完成在零位的关节扭矩传感器清零；
# 2. 动力学辨识参数辨识。
# ---------------------------------------------------------------------------


from __future__ import annotations
import os
import sys
from time import sleep
_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core.client import ArmClient
# from demo.arm_status_report import log_mechunit_ino_state, log_robot_state, log_snapshot
from core.types import sdk_status_message
from core import logger

# ------------------------------------------------------------------ 运行参数（按需修改）
HOST = "192.168.23.30"
PORT = 8000

arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)

try:

    ret = arm.SetEnableState(0, 2)


except KeyboardInterrupt:
    logger.info("Ctrl+C 中断")
finally:
    arm.disconnect()
    logger.info("已断开连接")
