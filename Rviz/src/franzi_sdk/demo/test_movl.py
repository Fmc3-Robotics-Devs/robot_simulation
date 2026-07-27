#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
import os
import sys
from time import sleep
_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)
import math
from core.client import ArmClient
from demo.arm_status_report import log_mechunit_ino_state, log_robot_state, log_snapshot
from core.types import sdk_status_message
from core import logger
import math
import json

from typing import Any, Callable, List, Optional, Tuple
from core.types import (
    ArmMotionMode,
    ArmSide,
    ArmSystemState,
    EmergencyStopType,
    MechUnitStateType,
    SdkStatus,
    mech_unit_state_name,
    mech_unit_type_name,
    sdk_status_message,
)
def _rsp_status(rsp: Any) -> int:
    if isinstance(rsp, dict):
        return int(rsp.get("status", 0))
    return 0

def fmt_joints(vec: List[float], prec: int = 4) -> str:
    if not vec:
        return "[]"
    return "[" + ", ".join(f"{x:.{prec}f}" for x in vec) + "]"

# ------------------------------------------------------------------ 运行参数（按需修改）
HOST = "192.168.23.30"
PORT = 8000

arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)

def log_snapshot(
    arm: Any,
    log_fn: Callable[[str], None],
    sides: Tuple[ArmSide, ...] = (ArmSide.LEFT, ArmSide.RIGHT),
) -> None:
    """整机快照"""
    sep = "─" * 72

    log_fn(
        f"[SDK] version={arm.GetVersion()} IsReady={arm.IsReady()} "
        f"AxisCount={arm.AxisCount()}"
    )

    log_fn(
        "[电控柜] "
        f"急停={arm.GetCabinetEmergencyStop()} "
        f"电源raw={arm.GetCabinetPowerStateRaw()[1]} "
        f"异常码={arm.GetCabinetPowerExceptionCode()[1]} "
        f"风扇码={arm.GetCabinetFanFailureCode()[1]}"
    )

    for side in sides:
        key = side.name.lower()
        log_fn(sep)
        log_fn(f"[{key.upper()} 臂]")
        log_fn(f"  使能: {arm.GetEnableState(side)}")
        log_fn(f"  当前关节角(rad): {fmt_joints(arm.GetCurJointPos(side))}")
        log_fn(f"  当前关节速: {fmt_joints(arm.GetCurJointVel(side), 3)}")
        log_fn(f"  期望关节角: {fmt_joints(arm.GetDesireJointPos(side))}")
        log_fn(f"  速度比例(%): {arm.GetJointVelocityRatio(side)}")

        js = arm.ReadJointState(side)
        if _rsp_status(js) == 0:
            log_fn(f"  ReadJointState: pos={fmt_joints(js.get('position') or [])}")

        fs = arm.GetFaultState(side)
        if _rsp_status(fs) == 0:
            log_fn(f"  故障: {fs.get('joint_has_fault') or []}")

        log_fn(f"  笛卡尔位姿: {arm.ReadCartesianPose(side)}")
        log_fn(f"  六维力: {fmt_joints(arm.ReadCartesianWrench(side), 2)}")

    try:
        fsd = arm.ReadFullStateDual()
        if isinstance(fsd, dict) and _rsp_status(fsd) == 0:
            log_fn(sep)
            log_fn(
                "[双臂 ReadFullStateDual] "
                + json.dumps(fsd, ensure_ascii=False)[:200]
                + "..."
            )
    except Exception as ex:
        log_fn(f"[双臂 ReadFullStateDual] {type(ex).__name__}: {ex}")

    log_fn(f"[其它] user_control_loop={arm.IsUserControlLoopEnabled()}")


try:
    ret = arm.SetEnableState(0, 1)
    ret = arm.SetEnableState(10, 1)
    sleep(3)

    ret_left = arm.MoveJ(side=0, joint_path=[[0,0,0,0.4,0,0,0]])
    ret_right = arm.MoveJ(side=1, joint_path=[[0,0,0,0.4,0,0,0]])
    sleep(2)
    # log_snapshot(arm, logger.info, sides=(ArmSide.LEFT,))



    # MoveL：绝对 6D 路点（m + rad），不含起点；来自 test_local/cases/left_arm_line.json
    # 前提：当前关节角接近 [0,0,0,0.4,0,0,0] rad，否则需按 FK 重算路点
    left_cartesian_path = [
         [0.5244, 0.1483, 0,0,-0.5236,-1.57],
         [0.08, 0.309, 0, 2.3562,-1.57,0.78],
        #  [0.18, 0.309, -0.1, 2.3562,-1.57,0.78],
        [0.5244, 0.1483, 0,0,-0.5236,-1.57],
    ]
    right_cartesian_path = [
         [0.5244, 0.1483, 0,0,-0.5236,-1.57],
         [0.08, 0.309, 0, 2.3562,-1.57,0.78],
        #  [0.18, 0.309, 0.1, 2.3562,-1.57,0.78],
        [0.5244, 0.1483, 0,0,-0.5236,-1.57],
    ]
    movel_cart_path = left_cartesian_path
    count = 0
    side=0
    ratio = 100
    rc = arm.SetJointVelocityRatio(side, ratio)
    logger.info("SetJointVelocityRatio(side=%d, %d%%) rc=%s", side, ratio, sdk_status_message(rc))
    while count < 10:
        count += 1
        rc_l= arm.MoveL(cartesian_path=movel_cart_path, side=side)
        # rc_r = arm.MoveL(cartesian_path=right_cartesian_path, side=1)
        # sleep(2)
    logger.info("Movl rc_l=%s %s", rc_l, sdk_status_message(rc_l))
    # logger.info("Movl rc_r=%s %s", rc_r, sdk_status_message(rc_r))
    sleep(2)
    # log_snapshot(arm, logger.info, sides=(ArmSide.LEFT,))

except KeyboardInterrupt:
    logger.info("Ctrl+C 中断")
finally:
    arm.disconnect()
    logger.info("已断开连接")
