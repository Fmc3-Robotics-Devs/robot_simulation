#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
import os
import sys
from time import sleep
_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core.client import ArmClient
from demo.arm_status_report import log_mechunit_ino_state, log_robot_state, log_snapshot
from core.types import sdk_status_message
from core import logger

# ------------------------------------------------------------------ 运行参数（按需修改）
HOST = "192.168.23.30"
PORT = 8000

arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)

try:
    # MechUnitModule INO：GetSystemState / ReadJointState / ReadFullStateDual 等
    # log_mechunit_ino_state(arm, lambda msg: logger.info("%s", msg))

    ret = arm.SetEnableState(10, 1)
    sleep(2)
    # # ret = arm.SetEnableState(1, 1)

    # logger.info("SetEnableState rc=%s %s", ret, sdk_status_message(ret))

    # log_snapshot(arm, lambda msg: logger.info("[snapshot] %s", msg))


    # logger.info("PushJointTeleopQueue left pt%d rc=%s %s", i, rc, sdk_status_message(rc))


    # path = [[0.3] * 7, [0.0] * 7]
    # loop_count = 0
    # while True:
    #     loop_count += 1
    #     ret_left = arm.MoveJ(side=0, joint_path=path)
    #     # ret_right = arm.MoveJ(side=1, joint_path=path)
    #     # logger.info(f"[loop {loop_count}] MoveJ left: {ret_left},MoveJ right: {ret_right}")
    #     sleep(2)


except KeyboardInterrupt:
    logger.info("Ctrl+C 中断")
finally:
    arm.disconnect()
    logger.info("已断开连接")
