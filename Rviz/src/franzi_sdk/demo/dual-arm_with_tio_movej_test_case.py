#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双臂上使能 + MoveJ 关节运动 + 双夹爪同步开合 融合Demo
功能：
1. 左右臂机械单元下电→上电使能
2. 双臂同步关节运动 MoveJ 往返
3. 左右臂双夹爪初始化、同步开合循环
4. 急停检测、状态打印、Ctrl+C安全断开释放
"""
from __future__ import annotations

import os
import sys
import time

# -------------------------- SDK 路径加载 --------------------------
_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core import logger
from core.client import ArmClient
from core.types import ArmSide, MechUnitLifecycleCmd, MechUnitType, SdkStatus, mech_unit_state_name, sdk_status_message

# ==============================================================================
# 全局运行配置（统一IP端口，两处代码合并共用）
# ==============================================================================
HOST = "192.168.23.30"
PORT = 8000
CMD_TIMEOUT_MS = 1000

# 机械臂单元选择
LEFT_ARM = MechUnitType.LEFTARM
RIGHT_ARM = MechUnitType.RIGHTARM

# MoveJ 目标关节角度 rad
JOINT_UP = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
JOINT_ZERO = [0, 0, 0, 0.0, 0, 0, 0]

# ==============================================================================
# 夹爪配置区域
# ==============================================================================
# 测试模式切换：MODE_SINGLE_GRIP / MODE_DUAL_GRIP
TEST_MODE = "MODE_DUAL_GRIP"

# 夹爪位置与力参数
OPEN_POS = 1000
CLOSE_POS = 0
FORCE = 40

# 两路夹爪配置，默认映射为 LEFT->TIO0、RIGHT->TIO1
grip1_cfg = {
    "side": ArmSide.LEFT,
    "slave_id": 1,
    "open_pos": OPEN_POS,
    "close_pos": CLOSE_POS,
    "force": FORCE,
    "name": "左臂夹爪"
}
grip2_cfg = {
    "side": ArmSide.RIGHT,
    "slave_id": 1,
    "open_pos": OPEN_POS,
    "close_pos": CLOSE_POS,
    "force": FORCE,
    "name": "右臂夹爪"
}

# 根据模式加载夹爪列表
if TEST_MODE == "MODE_SINGLE_GRIP":
    all_grips = [grip1_cfg]
    logger.info("【夹爪模式】单夹爪测试 MODE_SINGLE_GRIP")
elif TEST_MODE == "MODE_DUAL_GRIP":
    all_grips = [grip1_cfg, grip2_cfg]
    logger.info("【夹爪模式】双夹爪同步测试 MODE_DUAL_GRIP")
else:
    raise RuntimeError("TEST_MODE 配置错误，请选择 MODE_SINGLE_GRIP / MODE_DUAL_GRIP")

# ==============================================================================
# 工具函数：机械单元状态打印
# ==============================================================================
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
        logger.info("机械单元状态: %s", mech_unit_state_name(unit_state))

# ==============================================================================
# 工具函数：夹爪状态打印
# ==============================================================================
def print_grip_status(arm: ArmClient, grip, tag: str):
    name = grip["name"]
    side = grip["side"]
    sid = grip["slave_id"]
    logger.info(f"\n===== [{name}] {tag} =====")
    pos = arm.AgReadPosition(side, rs485_slave_id=sid)
    force = arm.AgReadForce(side, rs485_slave_id=sid)
    grip_state = arm.AgReadGripState(side, rs485_slave_id=sid)
    logger.info(f"位置 pos: {pos}")
    logger.info(f"实时力 force: {force}")
    logger.info(f"夹持状态 grip: {grip_state}")

# ==============================================================================
# 工具函数：单夹爪初始化等待就绪
# ==============================================================================
def single_grip_init_wait(arm: ArmClient, grip, max_loop=100, sleep_dt=0.1):
    name = grip["name"]
    side = grip["side"]
    sid = grip["slave_id"]
    force_val = grip["force"]

    # 执行夹爪初始化
    init_rsp = arm.AgInitialize(side, rs485_slave_id=sid, full_calibration=False)
    logger.info(f"\n[{name}] 初始化指令返回: {init_rsp}")
    time.sleep(0.2)

    # 设置夹持力
    force_rsp = arm.AgSetForce(
        side, force_val,
        rs485_slave_id=sid,
        timeout_ms=CMD_TIMEOUT_MS
    )
    logger.info(f"[{name}] 设置夹持力{force_val}返回: {force_rsp}")
    time.sleep(0.2)

    # 轮询等待初始化完成
    ready = False
    for i in range(max_loop):
        rsp = arm.AgReadInitState(side, rs485_slave_id=sid)
        logger.info(f"[{name}] 初始化状态轮询{i}: {rsp}")
        if rsp.get("status") == 0 and rsp.get("value") == 1:
            ready = True
            break
        time.sleep(sleep_dt)

    if not ready:
        logger.error(f"\n[{name}] 初始化等待超时，程序退出！")
        arm.disconnect()
        exit(1)
    logger.info(f"[{name}] 初始化完成就绪")

# ==============================================================================
# 主程序逻辑
# ==============================================================================
def main():
    arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)
    try:
        # 1. 上电前状态打印
        logger.info("========== 上电前机械单元状态 ==========")
        log_mechunit_state(arm, LEFT_ARM)
        log_mechunit_state(arm, RIGHT_ARM)

        # 2. 急停检测
        estop = arm.GetEmergencyStopState()
        logger.info(
            "急停状态: status=%s estop_active=%s reg=0x%04x ts_us=%s",
            estop.get("status"),
            estop.get("estop_active"),
            int(estop.get("estop_reg", 0)),
            estop.get("timestamp_us"),
        )
        if arm.IsEmergencyStopActive():
            logger.warning("检测到急停有效，跳过机械臂使能流程，仅初始化夹爪")
        else:
            # 3. 双臂先下电再上电使能
            logger.info("\n========== 执行双臂下电 DISABLE ==========")
            ret = arm.SetMechUnitLifecycle(LEFT_ARM, MechUnitLifecycleCmd.DISABLE)
            time.sleep(1)
            ret = arm.SetMechUnitLifecycle(RIGHT_ARM, MechUnitLifecycleCmd.DISABLE)
            time.sleep(1)

            logger.info("\n========== 执行双臂上电 ENABLE ==========")
            ret = arm.SetMechUnitLifecycle(LEFT_ARM, MechUnitLifecycleCmd.ENABLE)
            time.sleep(1)
            ret = arm.SetMechUnitLifecycle(RIGHT_ARM, MechUnitLifecycleCmd.ENABLE)
            time.sleep(1)

            logger.info("========== 上电后机械单元状态 ==========")
            log_mechunit_state(arm, LEFT_ARM)
            log_mechunit_state(arm, RIGHT_ARM)

        # 4. 初始化所有夹爪
        logger.info("\n========== 开始初始化夹爪 ==========")
        for grip in all_grips:
            single_grip_init_wait(arm, grip)
        time.sleep(0.5)
        logger.info("========== 夹爪全部就绪，进入运动+夹爪同步循环 ==========")

        # 5. 主循环：MoveJ 双臂运动 + 夹爪同步开合
        while True:
            # -------------------------- 第一步：双臂抬臂 MoveJ --------------------------
            logger.info("\n---------- MoveJ 双臂抬臂至 JOINT_UP ----------")
            ret_left = arm.MoveJ(side=ArmSide.LEFT, joint_path=[JOINT_UP])
            ret_right = arm.MoveJ(side=ArmSide.RIGHT, joint_path=[JOINT_UP])
            logger.info(f"左臂MoveJ返回: {ret_left}, 右臂MoveJ返回: {ret_right}")
            time.sleep(2)

            # -------------------------- 同步打开所有夹爪 --------------------------
            logger.info("\n---------- 执行夹爪全部打开 ----------")
            for grip in all_grips:
                rsp = arm.AgSetPosition(
                    grip["side"], grip["open_pos"],
                    rs485_slave_id=grip["slave_id"],
                    timeout_ms=CMD_TIMEOUT_MS
                )
                logger.info(f"{grip['name']} open指令返回: {rsp}")
            time.sleep(0.1)
            for grip in all_grips:
                print_grip_status(arm, grip, "打开后状态")
            time.sleep(1.9)

            # -------------------------- 第二步：双臂回零 MoveJ --------------------------
            logger.info("\n---------- MoveJ 双臂回零 JOINT_ZERO ----------")
            ret_left = arm.MoveJ(side=ArmSide.LEFT, joint_path=[JOINT_ZERO])
            ret_right = arm.MoveJ(side=ArmSide.RIGHT, joint_path=[JOINT_ZERO])
            logger.info(f"左臂MoveJ返回: {ret_left}, 右臂MoveJ返回: {ret_right}")
            time.sleep(2)

            # -------------------------- 同步闭合所有夹爪 --------------------------
            logger.info("\n---------- 执行夹爪全部闭合 ----------")
            for grip in all_grips:
                rsp = arm.AgSetPosition(
                    grip["side"], grip["close_pos"],
                    rs485_slave_id=grip["slave_id"],
                    timeout_ms=CMD_TIMEOUT_MS
                )
                logger.info(f"{grip['name']} close指令返回: {rsp}")
            time.sleep(0.1)
            for grip in all_grips:
                print_grip_status(arm, grip, "闭合后状态")
            time.sleep(1.9)

    except KeyboardInterrupt:
        logger.info("\n\n===== 检测到 Ctrl+C 中断程序 =====")
    finally:
        # 安全释放连接
        arm.disconnect()
        logger.info("已断开机械臂客户端连接，程序结束")

if __name__ == "__main__":
    main()
