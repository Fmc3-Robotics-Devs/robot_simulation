#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""查询机械单元 / 系统状态 demo — 仅使用 SystemModule 提供的 RPC。"""

from __future__ import annotations

import os
import sys
from typing import Iterable

_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core import logger
from core.client import ArmClient
from core.types import (
    ArmSide,
    ArmSystemState,
    MechUnitType,
    SdkStatus,
    format_mechunit_error_code,
    mech_unit_state_name,
    mech_unit_type_name,
    sdk_status_message,
)

# ------------------------------------------------------------------ 运行参数（按需修改）
HOST = "192.168.23.30"
PORT = 8000
COMPONENTS: Iterable[MechUnitType] = (MechUnitType.LEFTARM, MechUnitType.RIGHTARM)


def fmt_system_state(st: ArmSystemState) -> str:
    zh = {
        ArmSystemState.MOVING: "运动中",
        ArmSystemState.EMERGENCY_STOP: "急停",
        ArmSystemState.IDLE: "空闲",
        ArmSystemState.NOT_ENABLED: "未使能",
    }.get(st, "")
    return f"{st.name}({int(st)}) {zh}".strip()


def log_system_state(arm: ArmClient) -> None:
    logger.info("======== SystemState ========")
    for side in (ArmSide.BIO, ArmSide.LEFT, ArmSide.RIGHT):
        try:
            st = arm.GetSystemState(side)
            logger.info("GetSystemState(%s): %s", side.name, fmt_system_state(st))
        except Exception as ex:
            logger.info("GetSystemState(%s): %s: %s", side.name, type(ex).__name__, ex)

        try:
            en = arm.GetEnableState(side)
            logger.info("GetEnableState(%s): %s", side.name, en)
        except Exception as ex:
            logger.info("GetEnableState(%s): %s: %s", side.name, type(ex).__name__, ex)
    logger.info("======== SystemState end ========")


def log_mechunit_state(arm: ArmClient, component_type: int) -> None:
    ct = int(component_type)
    logger.info("======== MechUnit (%s) ========", mech_unit_type_name(ct))

    try:
        st, unit_state = arm.GetMechUnitState(ct)
    except Exception as ex:
        logger.info("GetMechUnitState: %s: %s", type(ex).__name__, ex)
        logger.info("======== MechUnit end ========")
        return

    if st != SdkStatus.OK:
        logger.info("GetMechUnitState: status=%s %s", st, sdk_status_message(st))
    else:
        logger.info("GetMechUnitState: %s", mech_unit_state_name(unit_state))

    try:
        info = arm.GetMechUnitErrorInfo(ct)
    except Exception as ex:
        logger.info("GetMechUnitErrorInfo: %s: %s", type(ex).__name__, ex)
        logger.info("======== MechUnit end ========")
        return

    st_info = int(info.get("status", 0))
    if st_info != SdkStatus.OK:
        logger.info(
            "GetMechUnitErrorInfo: status=%s %s rsp=%s",
            st_info,
            sdk_status_message(st_info),
            info,
        )
        logger.info("======== MechUnit end ========")
        return

    logger.info(
        "GetMechUnitErrorInfo: mech_state=%s unit_has_fault=%s "
        "fault_joint_count=%s primary_error_code=%s primary_joint_index=%s",
        mech_unit_state_name(int(info.get("mech_state", 0))),
        info.get("unit_has_fault"),
        info.get("fault_joint_count"),
        format_mechunit_error_code(int(info.get("primary_error_code", 0))),
        info.get("primary_joint_index"),
    )
    for j in info.get("joints") or []:
        if int(j.get("has_fault", 0)):
            logger.info(
                "  joint[%s] error_code=%s servo_state=%s follow_err=%s",
                j.get("joint_index"),
                format_mechunit_error_code(int(j.get("error_code", 0))),
                j.get("servo_state"),
                j.get("follow_error_active"),
            )

    logger.info("======== MechUnit end ========")


arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)

try:
    log_system_state(arm)
    for component in COMPONENTS:
        log_mechunit_state(arm, int(component))

except KeyboardInterrupt:
    logger.info("Ctrl+C 中断")
finally:
    arm.disconnect()
    logger.info("已断开连接")
