#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
import os
import sys
from time import sleep, perf_counter
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
from core.types import MechUnitLifecycleCmd, MechUnitType, SdkStatus, mech_unit_state_name, sdk_status_message


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
_MOTION_STATUS_ZH = {
    0: "停止且到位",
    1: "运动中",
    2: "停止未到位",
}
def fmt_motion_status(code: int) -> str:
    zh = _MOTION_STATUS_ZH.get(int(code), "未知")
    return f"{int(code)} ({zh})"
    
def wait_motion_arrived(
    arm: ArmClient,
    side: int,
    *,
    timeout_s: float = 600.0,
    poll_interval_s: float = 0.05,
) -> bool:
    """轮询 GetMotionStatus，直到返回 0（停止且到位）或超时。

    返回值语义（见 feedback.GetMotionStatus）:
      0 — 停止且到位
      1 — 运动中
      2 — 停止未到位（继续轮询，等待稳定到位）
    """
    deadline = perf_counter() + timeout_s
    last_status = -1

    while perf_counter() < deadline:
        status = arm.GetMotionStatus(side)
        if status != last_status:
            logger.info("GetMotionStatus(side=%d) -> %s", side, fmt_motion_status(status))
            last_status = status
        if status == 0:
            return True
        sleep(poll_interval_s)

    logger.error(
        "等待运动到位超时 (%.1fs)，最后状态: %s",
        timeout_s,
        fmt_motion_status(last_status),
    )
    return False

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
    logger.info("--- start test ---")
    log_mechunit_state(arm, MechUnitType.LEFTARM)
    log_mechunit_state(arm, MechUnitType.RIGHTARM)
    log_mechunit_state(arm, MechUnitType.WAIST)
    log_mechunit_state(arm, MechUnitType.HEAD)
    ret = arm.SetMechUnitLifecycle(MechUnitType.LEFTARM, MechUnitLifecycleCmd.DISABLE)
    ret = arm.SetMechUnitLifecycle(MechUnitType.RIGHTARM, MechUnitLifecycleCmd.DISABLE)
    ret = arm.SetMechUnitLifecycle(MechUnitType.WAIST, MechUnitLifecycleCmd.DISABLE)
    ret = arm.SetMechUnitLifecycle(MechUnitType.HEAD, MechUnitLifecycleCmd.DISABLE)
    sleep(1)
    ret = arm.SetMechUnitLifecycle(MechUnitType.LEFTARM, MechUnitLifecycleCmd.RESET)
    ret = arm.SetMechUnitLifecycle(MechUnitType.RIGHTARM, MechUnitLifecycleCmd.RESET)
    ret = arm.SetMechUnitLifecycle(MechUnitType.WAIST, MechUnitLifecycleCmd.RESET)
    ret = arm.SetMechUnitLifecycle(MechUnitType.HEAD, MechUnitLifecycleCmd.RESET)
    sleep(1)
    ret = arm.SetMechUnitLifecycle(MechUnitType.WAIST, MechUnitLifecycleCmd.ENABLE)
    ret = arm.SetMechUnitLifecycle(MechUnitType.HEAD, MechUnitLifecycleCmd.ENABLE)
    ret = arm.SetMechUnitLifecycle(MechUnitType.LEFTARM, MechUnitLifecycleCmd.ENABLE)
    ret = arm.SetMechUnitLifecycle(MechUnitType.RIGHTARM, MechUnitLifecycleCmd.ENABLE)
    sleep(1)
    logger.info("--- after test ---")
    log_mechunit_state(arm, MechUnitType.LEFTARM)
    log_mechunit_state(arm, MechUnitType.RIGHTARM)
    log_mechunit_state(arm, MechUnitType.WAIST)
    log_mechunit_state(arm, MechUnitType.HEAD)
    import math
    k=math.pi/180
    #side 0:代表左臂，1代表右臂，2代表升降机构，3代表头部
    ret_head = arm.MoveJ(joint_path=[[0.0,math.pi/4]], side=3)
    ret_waist = arm.MoveJ(joint_path=[[-0.8, 1.6, -0.8, 0]], side=2)
    ret_waist = arm.MoveJ(joint_path=[[0, 0, 0, 0]], side=2)
    ret_left = arm.MoveJ(joint_path=[[0,0,0,0,0,0,0]], side=0)
    ret_right = arm.MoveJ(joint_path=[[0,0,0,0,0,0,0]], side=1)
    ret_left = arm.MoveJ(joint_path=[[0,0,0,90*k,0,0,0]], side=0)
    ret_right = arm.MoveJ(joint_path=[[0,0,0,90*k,0,0,0]], side=1)
    ret_left = arm.MoveJ(joint_path=[[0,0,0,0,0,0,0]], side=0)
    ret_right = arm.MoveJ(joint_path=[[0,0,0,0,0,0,0]], side=1)
   


except KeyboardInterrupt:
    logger.info("Ctrl+C 中断")
finally:
    arm.disconnect()
    logger.info("已断开连接")
