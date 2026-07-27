
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
import os
import sys

_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core.client import ArmClient
from core.types import sdk_status_message
from core import logger


# ---------------------------------------------------------------------------
# Python SDK 测试用例：MoveJ 回零位测试
# ---------------------------------------------------------------------------
# 测试目的：
# 1. 验证 MoveJ 接口能否正常下发。
# 2. 验证机器人能否通过关节空间运动到零位。
#
# 使用前请确认：
# 1. 机器人控制程序和 SDK Server 已经启动；
# 2. 机器人周围环境安全，急停按钮可用；
# 3. 当前机器人姿态运动到零位不会发生碰撞或超过关节限位。
#
# 注意事项：
# 1. 本测试用例默认测试右臂（side=1），如需测试左臂请将 side 修改为 0。
# 2. q_zero 为目标零位，单位为 rad。
# 3. 如果机器人当前姿态距离零位较远，请先确认运动路径安全后再运行。
# ---------------------------------------------------------------------------


HOST = "192.168.23.30"
PORT = 8000

arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)

# side=0 -> 左臂
# side=1 -> 右臂
side = 0

try:
    # 目标零位：7 个关节全部运动到 0 rad
    q_zero = [0, 0, 0, 0, 0, 0, 0]

    # MoveJ 的 joint_path 是关节轨迹点列表
    # 这里仅下发一个目标点，表示从当前姿态运动到零位
    joint_path = [q_zero]

    logger.info("MoveJ to zero position ...")
    ret = arm.MoveJ(side=side, joint_path=joint_path)
    logger.info("MoveJ ret=%s, msg=%s", ret, sdk_status_message(ret))

    if ret != 0:
        logger.error("MoveJ to zero position failed")
    else:
        logger.info("MoveJ to zero position command accepted.")

except KeyboardInterrupt:
    logger.info("Ctrl+C 中断")

finally:
    arm.disconnect()
    logger.info("已断开连接")

