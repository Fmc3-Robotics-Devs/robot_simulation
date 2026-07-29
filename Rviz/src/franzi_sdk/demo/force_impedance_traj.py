#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# ---------------------------------------------------------------------------
# Python SDK 力控测试用例 3：阻抗控制 + MoveJ 单点往返轨迹跟踪测试
# ---------------------------------------------------------------------------
# 测试目的：
# 1. 验证 StartImpedance 接口能否正常下发，使机器人进入关节阻抗控制模式。
# 2. 验证在阻抗控制模式下，MoveJ 单点轨迹能否正常执行，实现柔顺轨迹跟踪。
# 3. 验证程序异常退出或 Ctrl+C 中断时，能够自动调用 ExitImpedance，避免机器人一直停留在阻抗控制模式。
#
# 测试力控前请确认：
# 1. 机器人已在零位完成关节扭矩传感器清零；
# 2. 机器人动力学参数已完成辨识；
# 3. 机器人周围环境安全，测试过程中操作者应保持急停按钮可用。
#
# 参数修改说明：
# 1. side 用于选择测试机械臂，ArmSide.LEFT 表示左臂，ArmSide.RIGHT 表示右臂。
# 2. D_joint 为关节阻尼参数，阻尼越大，运动越不容易振荡，但响应会变慢。
# 3. K_joint 为关节刚度参数，刚度越大，轨迹跟踪越“硬”，但柔顺性会降低，并可能引起抖动、振荡或冲击风险，建议从小刚度开始逐步增加。
# 4. Deadzone_joint 为关节外力死区，数值越大，对小外力越不敏感；数值过小可能导致传感器噪声被放大，引起误动作或抖动。
# 5. repeat_times 用于设置往返运动次数，q_target 用于设置目标关节位置，单位为 rad。
#
# 注意事项：
# 1. 本测试用例默认测试右臂，如需测试左臂请将 side 修改为 ArmSide.LEFT。
# 2. 当前 J4 负限位为 20 deg，因此 q_start[3] 不能设置为 0.0。
# 3. 第一次测试时请使用较小的 q_target 和较低的 K_joint，确认机器人响应正常后再逐步增大。
# 4. 第一次测试时建议只设置 1-2 次往返，确认机器人响应正常后再增加往返次数。
# 5. MoveJ 当前速度限制在后端固定，测试前请确认 MoveJ 速度限制较低且安全。
# 6. 第一次测试建议给较大的 Deadzone_joint 以避免传感器噪声引起的误动作，确认机器人响应正常后再逐步减小死区。
# 7. 请确保 q_start/q_target 中的所有关节角度均在机器人关节限位范围内。
# ---------------------------------------------------------------------------


from __future__ import annotations

import math
import os
import sys
from time import sleep

# -------------------------- SDK 路径加载 --------------------------
_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core import logger
from core.client import ArmClient
from core.types import ArmSide, sdk_status_message


# ==============================================================================
# 全局运行配置
# ==============================================================================
HOST = "192.168.23.30"
PORT = 8000

DEG2RAD = math.pi / 180.0

# 默认测试右臂；如需测试左臂，改为 ArmSide.LEFT
side = ArmSide.RIGHT

# 阻抗参数
D_joint = [
    0.2, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1
]

K_joint = [
    70.0, 70.0, 70.0, 70.0, 70.0, 70.0, 0.0
]

Deadzone_joint = [
    1.7, 1.7, 1.2, 1.2, 1.2, 1.2, 1.2
]

# MoveJ 测试参数，单位 rad
repeat_times = 4
move_wait_s = 4.0

# 当前 J4 负限位为 20 deg，因此 q_start[3] 不能使用 0.0
q_start = [
    0.0,
    0.0,
    0.0,
    40.0 * DEG2RAD,
    0.0,
    0.0,
    0.0,
]

# 右臂目标点：
# 右臂 J2 限位为 [-180, 30] deg，因此 -0.8 rad 合法
q_target_right = [
    -0.3,
    -0.8,
    0.0,
    1.6,
    0.0,
    0.0,
    0.0,
]

# 左臂目标点：
# 左臂 J2 限位为 [-30, 180] deg，因此不要使用 -0.8 rad，这里改为 +0.8 rad
q_target_left = [
    -0.3,
    0.8,
    0.0,
    1.6,
    0.0,
    0.0,
    0.0,
]


# ==============================================================================
# 主程序逻辑
# ==============================================================================
def main():
    arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)

    impedance_enabled = False
    exit_impedance_called = False

    q_target = q_target_left if side == ArmSide.LEFT else q_target_right

    try:
        # ----------------------------------------------------------------------
        # 1. 急停状态检测
        # ----------------------------------------------------------------------
        estop = arm.GetEmergencyStopState()
        logger.info(
            "急停状态: status=%s estop_active=%s reg=0x%04x ts_us=%s",
            estop.get("status"),
            estop.get("estop_active"),
            int(estop.get("estop_reg", 0)),
            estop.get("timestamp_us"),
        )

        if arm.IsEmergencyStopActive():
            logger.error("检测到急停有效，跳过 StartImpedance 和 MoveJ 测试")
            return

        # ----------------------------------------------------------------------
        # 2. 进入阻抗控制模式
        # ----------------------------------------------------------------------
        logger.info("========== StartImpedance ==========")
        ret_imp = arm.StartImpedance(
            side=side,
            D_joint=D_joint,
            K_joint=K_joint,
            Deadzone_joint=Deadzone_joint,
        )
        logger.info("StartImpedance ret=%s, msg=%s", ret_imp, sdk_status_message(ret_imp))

        if ret_imp != 0:
            logger.error("StartImpedance failed, skip MoveJ")
            return

        impedance_enabled = True

        # ----------------------------------------------------------------------
        # 3. MoveJ 单点往返测试
        #
        # 因此这里每次只发送一个目标点，而不是一次性发送完整 joint_path。
        # ----------------------------------------------------------------------
        logger.info("========== Impedance + MoveJ tracking test ==========")
        logger.info("Move to q_start first ...")

        ret_movej = arm.MoveJ(side=side, joint_path=[q_start])
        logger.info("MoveJ q_start ret=%s, msg=%s", ret_movej, sdk_status_message(ret_movej))

        if ret_movej != 0:
            logger.error("MoveJ q_start failed, stop test")
            return

        sleep(move_wait_s)

        for i in range(repeat_times):
            logger.info(
                "\n---------- MoveJ cycle %d/%d: q_start -> q_target ----------",
                i + 1,
                repeat_times,
            )

            ret_movej = arm.MoveJ(side=side, joint_path=[q_target])
            logger.info("MoveJ q_target ret=%s, msg=%s", ret_movej, sdk_status_message(ret_movej))

            if ret_movej != 0:
                logger.error("MoveJ q_target failed at cycle %d", i + 1)
                break

            sleep(move_wait_s)

            logger.info(
                "\n---------- MoveJ cycle %d/%d: q_target -> q_start ----------",
                i + 1,
                repeat_times,
            )

            ret_movej = arm.MoveJ(side=side, joint_path=[q_start])
            logger.info("MoveJ q_start ret=%s, msg=%s", ret_movej, sdk_status_message(ret_movej))

            if ret_movej != 0:
                logger.error("MoveJ q_start failed at cycle %d", i + 1)
                break

            sleep(move_wait_s)

        logger.info("MoveJ impedance tracking test finished.")

    except KeyboardInterrupt:
        logger.info("\n\n===== 检测到 Ctrl+C 中断程序 =====")

    finally:
        # ----------------------------------------------------------------------
        # 4. 无论正常结束还是异常退出，都尝试退出阻抗控制模式
        # ----------------------------------------------------------------------
        if impedance_enabled and not exit_impedance_called:
            logger.info("Program exiting while Impedance is active, try to ExitImpedance ...")

            ret_exit = arm.ExitImpedance(side=side)
            logger.info("ExitImpedance ret=%s, msg=%s", ret_exit, sdk_status_message(ret_exit))

            if ret_exit == 0:
                impedance_enabled = False
                exit_impedance_called = True
                logger.info("ExitImpedance succeeded.")
            else:
                logger.error("ExitImpedance failed")

        arm.disconnect()
        logger.info("已断开连接，程序结束")


if __name__ == "__main__":
    main()