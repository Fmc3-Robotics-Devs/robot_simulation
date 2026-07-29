#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双臂笛卡尔遥操作测试：以 10Hz 推送 path_pos_v.csv 中的位姿序列。"""

from __future__ import annotations

import csv
import os
import sys
import time
from time import sleep

_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core.client import ArmClient
from core.types import MechUnitLifecycleCmd, MechUnitType, sdk_status_message
from core import logger

# ------------------------------------------------------------------ 运行参数（按需修改）
HOST = "192.168.23.30"
PORT = 8000
TELEOP_HZ = 10.0


def read_pose_csv(csv_path: str) -> list[list[float]]:
    pose_data: list[list[float]] = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pose_data.append(
                [
                    float(row["x"]),
                    float(row["y"]),
                    float(row["z"]),
                    float(row["rx"]),
                    float(row["ry"]),
                    float(row["rz"]),
                ]
            )
    return pose_data


def run_dual_teleop_at_hz(
    arm: ArmClient,
    left_poses: list[list[float]],
    right_poses: list[list[float]],
    hz: float,
) -> None:
    period_s = 1.0 / hz
    n = min(len(left_poses), len(right_poses))
    logger.info("PushCartesianTeleopQueueDual @ %.1f Hz, %d poses", hz, n)

    next_tick = time.perf_counter()
    for i in range(n):
        rc = arm.PushCartesianTeleopQueueDual(
            left_pose=left_poses[i],
            right_pose=right_poses[i],
        )
        if rc != 0 or i % int(hz) == 0:
            logger.info(
                "PushCartesianTeleopQueueDual pt=%d rc=%s %s left=%s right=%s",
                i,
                rc,
                sdk_status_message(rc),
                [round(v, 4) for v in left_poses[i]],
                [round(v, 4) for v in right_poses[i]],
            )
        if rc != 0:
            break

        next_tick += period_s
        delay = next_tick - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
        else:
            next_tick = time.perf_counter()


arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)

try:
    csv_path = os.path.join(os.path.dirname(__file__), "path_pos_v.csv")
    poses = read_pose_csv(csv_path)
    logger.info("loaded %d poses from %s", len(poses), csv_path)

    # ret = arm.SetEnableState(0, 1)
    # ret = arm.SetEnableState(10, 1)
    # sleep(1)

    cur_joints = [
        -0.001851563,
        -0.004080629,
        -4.79e-05,
        1.573342975,
        -0.010450244,
        3.60e-05,
        -4.79e-05,
    ]
    ret_left = arm.MoveJ(side=0, joint_path=[cur_joints])
    ret_right = arm.MoveJ(side=1, joint_path=[cur_joints])
    logger.info("MoveJ left=%s right=%s", ret_left, ret_right)
    sleep(6)

    ret_tele = arm.TeleCartesianDual()
    logger.info("TeleCartesianDual rc=%s %s", ret_tele, sdk_status_message(ret_tele))
    if ret_tele != 0:
        raise RuntimeError("TeleCartesianDual failed")

    # 左右臂使用同一路径；若有右臂专用 CSV，可单独加载后传入
    run_dual_teleop_at_hz(arm, poses, poses, TELEOP_HZ)

except KeyboardInterrupt:
    logger.info("Ctrl+C 中断")
finally:
    arm.StopTeleCartesianDual()
    arm.disconnect()
    logger.info("已断开连接")
