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

    ret = arm.SetEnableState(0, 1)
    # sleep(1)
    # ret = arm.SetEnableState(1, 1)

    logger.info("SetEnableState rc=%s %s", ret, sdk_status_message(ret))

    # log_snapshot(arm, lambda msg: logger.info("[snapshot] %s", msg))

    joint_path = [[0.0] * 7, [1.0] * 7]

    ret_left = arm.TeleJoint(side=0)
    logger.info("TeleJoint left rc=%s %s", ret_left, sdk_status_message(ret_left))
    if ret_left == 0:
        for i, joints in enumerate(joint_path):
            rc = arm.PushJointTeleopQueue(side=0, target_joints=joints)
            logger.info("PushJointTeleopQueue left pt%d rc=%s %s", i, rc, sdk_status_message(rc))

    ret_right = arm.TeleJoint(side=1)
    logger.info("TeleJoint right rc=%s %s", ret_right, sdk_status_message(ret_right))
    if ret_right == 0:
        for i, joints in enumerate(joint_path):
            rc = arm.PushJointTeleopQueue(side=1, target_joints=joints)
            logger.info("PushJointTeleopQueue right pt%d rc=%s %s", i, rc, sdk_status_message(rc))

except KeyboardInterrupt:
    logger.info("Ctrl+C 中断")
finally:
    arm.disconnect()
    logger.info("已断开连接")
