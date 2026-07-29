#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ---------------------------------------------------------------------------
# Python SDK 力控测试用例：拖动示教轨迹回放测试
# ---------------------------------------------------------------------------
# 测试目的：
# 1. 验证 Playback 接口能否正常下发；
# 2. 验证服务端能否从 /inodata/dragmode_joint_trajectory.txt 读取拖动示教记录轨迹；
# 3. 验证机器人能否先 MoveJ 到记录轨迹第一个点，然后按控制周期回放 q_d。
#
# 测试力控前请确认：
# 1. 机器人已经完成关节扭矩传感器清零；
# 2. 动力学辨识参数已经正确加载；
# 3. 已经通过 StartDragMode(record_enable=True) 生成轨迹文件：
#      /inodata/dragmode_joint_trajectory.txt
# 4. 机器人周围环境安全，急停可用；
# 5. 当前默认测试右臂 side=1，如需测试左臂请改为 side=0。
#
# 注意事项：
# 1. Playback 不从 Python 侧传入轨迹点，轨迹文件由控制侧读取；
# 2. Playback 启动后，服务端会执行 MoveJ 到轨迹第一个点，再启动实时回放；
# 3. 当前脚本只负责启动回放任务，不负责中途停止回放。
# ---------------------------------------------------------------------------

from __future__ import annotations

import os
import sys
from time import sleep

_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core.client import ArmClient
from core.types import sdk_status_message
from core import logger

# ------------------------------------------------------------------ 运行参数（按需修改）
HOST = "192.168.23.30"
PORT = 8000

# side=0：左臂
# side=1：右臂
SIDE = 0

# 回放启动后，Python 脚本保持连接的时间。
# 实际回放时长取决于 /inodata/dragmode_joint_trajectory.txt 中的采样点数量。
PLAYBACK_WAIT_SECONDS = 20.0

arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)

try:
    logger.info("准备启动拖动示教轨迹回放，side=%s", SIDE)
    logger.info("服务端默认读取轨迹文件: /inodata/dragmode_joint_trajectory.txt")

    ret = arm.Playback(side=SIDE)
    logger.info("Playback 返回: ret=%s, message=%s", ret, sdk_status_message(ret))

    if ret != 0:
        logger.error("Playback 启动失败，请检查服务端日志和轨迹文件是否存在")
    else:
        logger.info("Playback 启动成功，等待回放执行中...")
        sleep(PLAYBACK_WAIT_SECONDS)
        logger.info("等待结束，如轨迹较长请适当增大 PLAYBACK_WAIT_SECONDS")

except KeyboardInterrupt:
    logger.info("Ctrl+C 中断")

finally:
    arm.disconnect()
    logger.info("已断开连接")