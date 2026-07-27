#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""上使能 demo — 经 SystemModule ``SetMechUnitLifecycle`` 下发 Enable。"""

from __future__ import annotations

import os
import sys

_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)
from time import sleep
from core import logger
from core.client import ArmClient
from core.types import MechUnitLifecycleCmd, MechUnitType, SdkStatus, mech_unit_state_name, sdk_status_message

# ------------------------------------------------------------------ 运行参数（按需修改）
HOST = "192.168.23.30"
PORT = 8000


def log_mechunit_state(arm: ArmClient, component_type: int) -> None:
    ct = int(component_type)
    try:
        st, unit_state = arm.GetMechUnitState(ct)
    except Exception as ex:
        logger.info("GetMechUnitState: %s: %s", type(ex).__name__, ex)
        return

    if st != SdkStatus.OK:
        logger.info("GetMechUnitState: status=%s %s", st, sdk_status_message(st))
    else:
        logger.info("GetMechUnitState: %s", mech_unit_state_name(unit_state))


arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)

try:
    logger.info("--- before enable ---")
    log_mechunit_state(arm, MechUnitType.LEFTARM)
    log_mechunit_state(arm, MechUnitType.RIGHTARM)

    ret = arm.SetMechUnitLifecycle(MechUnitType.LEFTARM, MechUnitLifecycleCmd.ENABLE)
    sleep(1)
    ret = arm.SetMechUnitLifecycle(MechUnitType.RIGHTARM, MechUnitLifecycleCmd.ENABLE)
    sleep(1)
    logger.info("--- after enable ---")
    log_mechunit_state(arm, MechUnitType.LEFTARM)
    log_mechunit_state(arm, MechUnitType.RIGHTARM)
    logger.info("--- start_running ---")
    for i in range(5):
        ret_left = arm.MoveJ(side=0, joint_path=[[0,0,0,1.5,0,0,0]])
        ret_right = arm.MoveJ(side=1, joint_path=[[0,0,0,1.5,0,0,0]])
        sleep(2)
        ret_left = arm.MoveJ(side=0, joint_path=[[0,0,0,0.4,0,0,0]])
        ret_right = arm.MoveJ(side=1, joint_path=[[0,0,0,0.4,0,0,0]])
        sleep(2)
    logger.info("--- run finished ---")
    ret = arm.SetMechUnitLifecycle(MechUnitType.LEFTARM, MechUnitLifecycleCmd.DISABLE)
    ret = arm.SetMechUnitLifecycle(MechUnitType.RIGHTARM, MechUnitLifecycleCmd.DISABLE)
except KeyboardInterrupt:
    logger.info("Ctrl+C 中断")
finally:
    arm.disconnect()
    logger.info("已断开连接")
