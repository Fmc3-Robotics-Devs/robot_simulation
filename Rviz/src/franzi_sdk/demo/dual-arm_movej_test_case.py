#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双臂上使能 + MoveJ 关节运动 Demo (已屏蔽全部TIO夹爪逻辑)
功能：
1. 左右臂机械单元下电→上电使能
2. 双臂同步关节运动 MoveJ 往返
3. 急停检测、状态打印、Ctrl+C安全断开释放
【修改说明】所有TIO/ag夹爪相关代码全部注释屏蔽,不再执行夹爪初始化、开合、状态读取
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
# 工具函数：机械单元状态打印（保留）
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
            logger.warning("检测到急停有效，跳过机械臂使能流程，无夹爪逻辑")
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

        # 4. 跳过夹爪初始化
        logger.info("========== 跳过夹爪初始化，直接进入MoveJ运动循环 ==========")

        # 5. 主循环：仅双臂MoveJ往返运动，移除所有夹爪开合代码
        while True:
            # -------------------------- 第一步：双臂抬臂 MoveJ --------------------------
            logger.info("\n---------- MoveJ 双臂抬臂至 JOINT_UP ----------")
            ret_left = arm.MoveJ(side=ArmSide.LEFT, joint_path=[JOINT_UP])
            ret_right = arm.MoveJ(side=ArmSide.RIGHT, joint_path=[JOINT_UP])
            logger.info(f"左臂MoveJ返回: {ret_left}, 右臂MoveJ返回: {ret_right}")
            time.sleep(4)  # 原等待时间合并夹爪延时，适当拉长保证运动到位

            # -------------------------- 第二步：双臂回零 MoveJ --------------------------
            logger.info("\n---------- MoveJ 双臂回零 JOINT_ZERO ----------")
            ret_left = arm.MoveJ(side=ArmSide.LEFT, joint_path=[JOINT_ZERO])
            ret_right = arm.MoveJ(side=ArmSide.RIGHT, joint_path=[JOINT_ZERO])
            logger.info(f"左臂MoveJ返回: {ret_left}, 右臂MoveJ返回: {ret_right}")
            time.sleep(4)

    except KeyboardInterrupt:
        logger.info("\n\n===== 检测到 Ctrl+C 中断程序 =====")
    finally:
        # 安全释放连接
        arm.disconnect()
        logger.info("已断开机械臂客户端连接，程序结束")

if __name__ == "__main__":
    main()
