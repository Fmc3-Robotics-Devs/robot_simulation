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
# COMPONENT = MechUnitType.RIGHTARM
COMPONENT = MechUnitType.LEFTARM
COMPONENT2 = MechUnitType.RIGHTARM

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

    estop = arm.GetEmergencyStopState()
    logger.info(
        "GetEmergencyStopState: status=%s estop_active=%s reg=0x%04x ts_us=%s",
        estop.get("status"),
        estop.get("estop_active"),
        int(estop.get("estop_reg", 0)),
        estop.get("timestamp_us"),
    )
    if arm.IsEmergencyStopActive():
        logger.info("急停有效，跳过 Enable")
    else:
        ret = arm.SetMechUnitLifecycle(COMPONENT, MechUnitLifecycleCmd.ENABLE)
        sleep(1)
        ret = arm.SetMechUnitLifecycle(COMPONENT2, MechUnitLifecycleCmd.ENABLE)     
        sleep(1)
        count = 0
        side=0
        ratio = 100
        rc = arm.SetJointVelocityRatio(side, ratio)
        logger.info("SetJointVelocityRatio(side=%d, %d%%) rc=%s", side, ratio, sdk_status_message(rc))
        while count < 10:
            count += 1
            ret_left = arm.MoveJ(joint_path=[[2.2,0,0.3,0.8,0.5,0.5,0.2],[0,0,0,0.,0,0,0]], side=side)
            ret_left = arm.MoveJ(joint_path=[[2.2,0,0.3,0.8,0.5,0.5,0.2]], side=side)


    logger.info("--- after enable ---")
    log_mechunit_state(arm, COMPONENT)

except KeyboardInterrupt:
    logger.info("Ctrl+C 中断")
finally:
    arm.disconnect()
    logger.info("已断开连接")
