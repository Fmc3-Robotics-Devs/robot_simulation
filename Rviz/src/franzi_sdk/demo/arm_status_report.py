# -*- coding: utf-8 -*-
"""
arm_status_report — 直接调 ArmClient 公开方法，格式化输出到日志。

建议 ``ArmClient(..., raise_on_error=False)``，单次 RPC 失败时不中断后续打印。
"""

from __future__ import annotations

import json
import math
import os
import sys
from typing import Any, Callable, List, Optional, Tuple

_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core.types import (
    ArmMotionMode,
    ArmSide,
    ArmSystemState,
    EmergencyStopType,
    MechUnitStateType,
    MechUnitType,
    SdkStatus,
    format_mechunit_error_code,
    mech_unit_state_name,
    mech_unit_type_name,
    sdk_status_message,
)

_MOTION_STATUS_ZH = {
    0: "静止",
    1: "运动中",
}


def fmt_system_state(st: Any) -> str:
    if isinstance(st, ArmSystemState):
        v = int(st)
        name = st.name
    else:
        v = int(st)
        try:
            name = ArmSystemState(v).name
        except ValueError:
            return f"UNKNOWN({v})"
    zh = {
        ArmSystemState.MOVING: "运动中",
        ArmSystemState.EMERGENCY_STOP: "急停",
        ArmSystemState.IDLE: "空闲",
        ArmSystemState.NOT_ENABLED: "未使能",
    }.get(ArmSystemState(v), "")
    return f"{name}({v}) {zh}"


def fmt_motion_mode(m: Any) -> str:
    if isinstance(m, ArmMotionMode):
        v = int(m)
        name = m.name
    else:
        v = int(m)
        try:
            name = ArmMotionMode(v).name
        except ValueError:
            return f"UNKNOWN({v})"
    return f"{name}({v})"


def fmt_estop(e: Any) -> str:
    if isinstance(e, EmergencyStopType):
        v = int(e)
        name = e.name
    else:
        v = int(e)
        try:
            name = EmergencyStopType(v).name
        except ValueError:
            return f"UNKNOWN({v})"
    zh = {
        EmergencyStopType.ZERO_TYPE: "0类急停-完全停止",
        EmergencyStopType.ONE_TYPE: "1类急停-保持位置",
        EmergencyStopType.TWO_TYPE: "2类急停-回初始",
        EmergencyStopType.NONE_TYPE: "无急停",
    }.get(EmergencyStopType(v), "")
    return f"{name}({v}) {zh}"


def fmt_motion_status(code: int) -> str:
    zh = _MOTION_STATUS_ZH.get(int(code), "")
    return f"{int(code)} {zh}".strip()


def fmt_joints(vec: List[float], prec: int = 4) -> str:
    if not vec:
        return "[]"
    return "[" + ", ".join(f"{x:.{prec}f}" for x in vec) + "]"


def fmt_rpc_status(st: Optional[int]) -> str:
    if st is None:
        return "—"
    if int(st) == SdkStatus.OK:
        return "OK(0)"
    return sdk_status_message(int(st))


def _rsp_status(rsp: Any) -> int:
    if isinstance(rsp, dict):
        return int(rsp.get("status", 0))
    return 0


def log_robot_state(arm: Any, log_fn: Callable[[str], None]) -> None:
    """MechUnit 整机状态：``arm.GetRobotState()``。"""
    log_fn("======== RobotState (MechUnit) ========")
    try:
        rsp = arm.GetRobotState()
    except Exception as ex:
        log_fn(f"  GetRobotState: {type(ex).__name__}: {ex}")
        log_fn("======== RobotState end ========")
        return

    st = int(rsp.get("status", 0))
    if st != SdkStatus.OK:
        log_fn(f"  GetRobotState: status={st} {fmt_rpc_status(st)} rsp={rsp}")
        log_fn("======== RobotState end ========")
        return

    log_fn(
        f"  unit_count={rsp.get('unit_count')} "
        f"command_authority={rsp.get('command_authority')} "
        f"timestamp_ns={rsp.get('timestamp_ns')}"
    )

    units = rsp.get("units") or []
    if not units:
        log_fn("  (no units)")
    for unit in units:
        unit_id = unit.get("unit_id") or "?"
        unit_type = int(unit.get("unit_type", 255))
        mech_state = int(unit.get("mech_state", 0))
        joint_count = int(unit.get("joint_count", 0))
        log_fn(
            f"  [{unit_id}] type={mech_unit_type_name(unit_type)} "
            f"mech_state={mech_unit_state_name(mech_state)} joints={joint_count}"
        )
        joints = unit.get("joints") or []
        if not joints:
            continue
        pos = [math.degrees(float(j.get("position_rad", 0.0))) for j in joints[:joint_count]]
        vel = [float(j.get("velocity_rad_s", 0.0)) for j in joints[:joint_count]]
        log_fn(f"    pos(deg)={fmt_joints(pos)}")
        log_fn(f"    vel(rad/s)={fmt_joints(vel, 3)}")
        if mech_state == int(MechUnitStateType.FAULT):
            codes = [
                format_mechunit_error_code(int(j.get("error_code", 0)))
                for j in joints[:joint_count]
            ]
            log_fn(f"    error_code={codes}")

    log_fn("======== RobotState end ========")


def log_mechunit_lifecycle(
    arm: Any,
    component_type: int,
    log_fn: Callable[[str], None],
) -> None:
    """SystemModule 机械单元生命周期：``GetMechUnitState`` / ``GetMechUnitErrorInfo``。"""
    ct = int(component_type)
    log_fn(f"======== MechUnitLifecycle ({mech_unit_type_name(ct)}) ========")

    try:
        st, unit_state = arm.GetMechUnitState(ct)
    except Exception as ex:
        log_fn(f"  GetMechUnitState: {type(ex).__name__}: {ex}")
        log_fn("======== MechUnitLifecycle end ========")
        return

    if st != SdkStatus.OK:
        log_fn(f"  GetMechUnitState: status={st} {fmt_rpc_status(st)}")
    else:
        log_fn(f"  GetMechUnitState: {mech_unit_state_name(unit_state)}")

    try:
        info = arm.GetMechUnitErrorInfo(ct)
    except Exception as ex:
        log_fn(f"  GetMechUnitErrorInfo: {type(ex).__name__}: {ex}")
        log_fn("======== MechUnitLifecycle end ========")
        return

    st_info = int(info.get("status", 0))
    if st_info != SdkStatus.OK:
        log_fn(f"  GetMechUnitErrorInfo: status={st_info} {fmt_rpc_status(st_info)} rsp={info}")
        log_fn("======== MechUnitLifecycle end ========")
        return

    log_fn(
        f"  GetMechUnitErrorInfo: mech_state={mech_unit_state_name(int(info.get('mech_state', 0)))} "
        f"unit_has_fault={info.get('unit_has_fault')} "
        f"fault_joint_count={info.get('fault_joint_count')} "
        f"primary_error_code={format_mechunit_error_code(int(info.get('primary_error_code', 0)))} "
        f"primary_joint_index={info.get('primary_joint_index')}"
    )
    joints = info.get("joints") or []
    for j in joints:
        if int(j.get("has_fault", 0)):
            log_fn(
                f"    joint[{j.get('joint_index')}] error_code="
                f"{format_mechunit_error_code(int(j.get('error_code', 0)))} "
                f"servo_state={j.get('servo_state')} follow_err={j.get('follow_error_active')}"
            )

    log_fn("======== MechUnitLifecycle end ========")


def log_emergency_stop_state(arm: Any, log_fn: Callable[[str], None]) -> None:
    """SystemModule 软急停：``GetEmergencyStopState`` / ``IsEmergencyStopActive``。"""
    log_fn("======== EmergencyStopState (SystemModule) ========")
    try:
        rsp = arm.GetEmergencyStopState()
    except Exception as ex:
        log_fn(f"  GetEmergencyStopState: {type(ex).__name__}: {ex}")
        log_fn("======== EmergencyStopState end ========")
        return

    st = int(rsp.get("status", 0))
    if st != SdkStatus.OK:
        log_fn(f"  GetEmergencyStopState: status={st} {fmt_rpc_status(st)} rsp={rsp}")
    else:
        active = bool(rsp.get("estop_active", False))
        log_fn(
            f"  estop_active={active} reg=0x{int(rsp.get('estop_reg', 0)):04x} "
            f"timestamp_us={rsp.get('timestamp_us')} "
            f"IsEmergencyStopActive={arm.IsEmergencyStopActive()}"
        )
    log_fn("======== EmergencyStopState end ========")


def log_mechunit_ino_state(arm: Any, log_fn: Callable[[str], None]) -> None:
    """MechUnit INO 相关状态：直接 ``arm.GetSystemState`` / ``ReadJointState`` 等。"""
    log_fn("======== MechUnit INO (system_state) ========")

    for side in (ArmSide.BIO, ArmSide.LEFT, ArmSide.RIGHT):
        try:
            val = arm.GetSystemState(side)
            log_fn(f"  GetSystemState({side.name}): {fmt_system_state(val)}")
        except Exception as ex:
            log_fn(f"  GetSystemState({side.name}): {type(ex).__name__}: {ex}")

        try:
            en = arm.GetEnableState(side)
            log_fn(f"  GetEnableState({side.name}): {en}")
        except Exception as ex:
            log_fn(f"  GetEnableState({side.name}): {type(ex).__name__}: {ex}")

    for side in (ArmSide.LEFT, ArmSide.RIGHT):
        js = arm.ReadJointState(side)
        st_js = _rsp_status(js)
        if st_js == 0:
            log_fn(
                f"  ReadJointState({side.name}): status=0 OK "
                f"pos={fmt_joints(js.get('position') or [])} "
                f"vel={fmt_joints(js.get('velocity') or [], 3)}"
            )
        else:
            log_fn(
                f"  ReadJointState({side.name}): status={st_js} "
                f"{fmt_rpc_status(st_js)} rsp={js}"
            )

        fs = arm.GetFaultState(side)
        st_fs = _rsp_status(fs)
        if st_fs == 0:
            log_fn(
                f"  GetFaultState({side.name}): status=0 OK "
                f"fault={fs.get('joint_has_fault')} code={fs.get('joint_fault_code')}"
            )
        else:
            log_fn(
                f"  GetFaultState({side.name}): status={st_fs} "
                f"{fmt_rpc_status(st_fs)} rsp={fs}"
            )

        full = arm.ReadFullState(side)
        st_full = _rsp_status(full)
        if st_full == 0:
            log_fn(
                f"  ReadFullState({side.name}): status=0 OK "
                f"pos={fmt_joints(full.get('joint_pos') or [])}"
            )
        else:
            log_fn(
                f"  ReadFullState({side.name}): status={st_full} "
                f"{fmt_rpc_status(st_full)} rsp={full}"
            )

    dual = arm.ReadFullStateDual()
    st_dual = _rsp_status(dual)
    if st_dual == 0 and isinstance(dual, dict):
        left = dual.get("left") or {}
        right = dual.get("right") or {}
        log_fn(
            "  ReadFullStateDual: status=0 OK "
            f"left_pos={fmt_joints(left.get('joint_pos') or [])} "
            f"right_pos={fmt_joints(right.get('joint_pos') or [])}"
        )
    else:
        log_fn(
            f"  ReadFullStateDual: status={st_dual} {fmt_rpc_status(st_dual)} "
            f"rsp={json.dumps(dual, ensure_ascii=False)[:240]}"
        )

    log_fn("======== MechUnit INO end ========")


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
        log_fn(f"  系统状态: {fmt_system_state(arm.GetSystemState(side))}")
        log_fn(f"  使能: {arm.GetEnableState(side)}")
        log_fn(f"  运动模式: {fmt_motion_mode(arm.GetMotionMode(side))}")
        log_fn(f"  急停类型: {fmt_estop(arm.GetEmergencyStopType(side))}")
        log_fn(f"  运动状态: {fmt_motion_status(arm.GetMotionStatus(side))}")
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


def log_robot_brief(arm: Any, log_fn: Callable[[str], None]) -> None:
    """简要状态：急停、使能、关节角、错误码（SystemModule + 关节反馈）。"""
    log_emergency_stop_state(arm, log_fn)

    for component in (MechUnitType.LEFTARM, MechUnitType.RIGHTARM):
        log_mechunit_lifecycle(arm, int(component), log_fn)

    log_fn("======== JointState ========")
    for side in (ArmSide.LEFT, ArmSide.RIGHT):
        try:
            pos = arm.GetCurJointPos(side)
            log_fn(f"  GetCurJointPos({side.name}) rad: {fmt_joints(pos)}")
        except Exception as ex:
            log_fn(f"  GetCurJointPos({side.name}): {type(ex).__name__}: {ex}")

        js = arm.ReadJointState(side)
        if _rsp_status(js) == 0:
            log_fn(
                f"  ReadJointState({side.name}) rad: "
                f"{fmt_joints(js.get('position') or [])}"
            )
        else:
            log_fn(f"  ReadJointState({side.name}): status={_rsp_status(js)} rsp={js}")
    log_fn("======== JointState end ========")


if __name__ == "__main__":
    from core import logger
    from core.client import ArmClient

    HOST = os.environ.get("INBC_SDK_HOST", "192.168.23.30")
    PORT = int(os.environ.get("INBC_SDK_PORT", "8000"))

    arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)
    try:
        logger.info("连接 %s:%s ...", HOST, PORT)
        if not arm.connect():
            logger.error("连接失败，请检查控制器 IP/端口及 SdkServerModule 是否已启动")
            sys.exit(1)

        log_fn = lambda msg: logger.info("%s", msg)
        log_fn(f"[连接成功] version={arm.GetVersion()} IsReady={arm.IsReady()}")

        # 急停 + 使能/错误码 + 关节角
        # log_robot_brief(arm, log_fn)
        log_robot_state(MechUnitType.LEFTARM, log_fn)
        # 可选：更完整的 MechUnit INO / 整机快照
        # log_mechunit_ino_state(arm, log_fn)
        # log_snapshot(arm, log_fn)
    except KeyboardInterrupt:
        logger.info("Ctrl+C 中断")
    finally:
        arm.disconnect()
        logger.info("已断开连接")
