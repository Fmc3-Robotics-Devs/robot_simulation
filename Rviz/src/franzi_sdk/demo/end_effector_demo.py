#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
直连串口 RS485 末端 Demo — 仅测末端（夹爪/吸盘），不含手臂运动。

CHANNELS 列表需与控制器 end_effector.yaml 中 channels 的 side + tool_type 一致。
左右均为夹爪时，两侧都填 tool="gripper"，循环会对每个夹爪分别 Open/Close。

运行：
  cd sdk/sdk_py && python3 demo/end_effector_demo.py
  python3 demo/end_effector_demo.py [host] [port]
"""

from __future__ import annotations

import sys
import time
from typing import Any, Dict, List

from core.arm_client import ArmClient
from core.types import ArmSide

HOST = "192.168.23.30"
PORT = 8000
CMD_TIMEOUT_MS = 1000

# 与 end_effector.yaml 对齐；左右都是夹爪示例：
CHANNELS: List[Dict[str, Any]] = [
    {"side": ArmSide.LEFT, "tool": "gripper", "name": "左夹爪"},
    {"side": ArmSide.RIGHT, "tool": "gripper", "name": "右夹爪"},
]
# CHANNELS: List[Dict[str, Any]] = [
#     {"side": ArmSide.LEFT, "tool": "gripper", "name": "左夹爪"},
# ]

# 左夹爪 + 右吸盘时改为：
# CHANNELS = [
#     {"side": ArmSide.LEFT, "tool": "gripper", "name": "左夹爪"},
#     {"side": ArmSide.RIGHT, "tool": "suction", "name": "右吸盘"},
# ]


def _host_port() -> tuple[str, int]:
    host = sys.argv[1] if len(sys.argv) > 1 else HOST
    port = int(sys.argv[2]) if len(sys.argv) > 2 else PORT
    return host, port


def _grippers() -> List[Dict[str, Any]]:
    return [ch for ch in CHANNELS if ch["tool"] == "gripper"]


def _suctions() -> List[Dict[str, Any]]:
    return [ch for ch in CHANNELS if ch["tool"] == "suction"]


def init_gripper(client: ArmClient, ch: Dict[str, Any]) -> None:
    side = ch["side"]
    name = ch["name"]
    print(f"\n========== [{name}] 夹爪初始化 ==========")
    print(f"GripperInitialize: {client.GripperInitialize(side, full_calibration=False, timeout_ms=CMD_TIMEOUT_MS)}")
    time.sleep(0.2)

    for i in range(100):
        rsp = client.GripperReadInitState(side, timeout_ms=CMD_TIMEOUT_MS)
        print(f"[{name}] init poll {i}: {rsp}")
        if rsp.get("status") == 0 and rsp.get("value") == 1:
            print(f"[{name}] 初始化完成")
            return
        time.sleep(0.1)
    print(f"[{name}] 警告：初始化轮询超时（yaml auto_initialize=true 时可能已完成）")


def print_gripper_status(client: ArmClient, ch: Dict[str, Any], tag: str) -> None:
    side = ch["side"]
    name = ch["name"]
    print(f"\n===== [{name}] {tag} =====")
    print(f"binary: {client.GripperGetBinaryState(side, timeout_ms=CMD_TIMEOUT_MS)}")
    print(f"position: {client.GripperReadPosition(side, timeout_ms=CMD_TIMEOUT_MS)}")
    print(f"grip_state: {client.GripperReadGripState(side, timeout_ms=CMD_TIMEOUT_MS)}")


def print_suction_status(client: ArmClient, ch: Dict[str, Any], tag: str) -> None:
    side = ch["side"]
    name = ch["name"]
    print(f"\n===== [{name}] {tag} =====")
    print(f"state: {client.SuctionReadState(side, timeout_ms=CMD_TIMEOUT_MS)}")


def main() -> None:
    host, port = _host_port()
    client = ArmClient(host, port, raise_on_error=False)
    if not client.connect():
        print(f"连接失败 {host}:{port}")
        sys.exit(1)
    print(f"已连接 {host}:{port}（仅末端，channels={len(CHANNELS)}）")

    for ch in _grippers():
        init_gripper(client, ch)
    time.sleep(0.5)

    grippers = _grippers()
    suctions = _suctions()
    print("\n========== 末端循环测试（Ctrl+C 退出）==========")
    try:
        while True:
            print("\n---------- 打开 / 松开 ----------")
            for ch in grippers:
                print(f"{ch['name']} GripperOpen: {client.GripperOpen(ch['side'], timeout_ms=CMD_TIMEOUT_MS)}")
            for ch in suctions:
                print(f"{ch['name']} SuctionRelease: {client.SuctionRelease(ch['side'], timeout_ms=CMD_TIMEOUT_MS)}")
            time.sleep(0.2)
            for ch in grippers:
                print_gripper_status(client, ch, "打开后")
            for ch in suctions:
                print_suction_status(client, ch, "松开后")
            time.sleep(1.8)

            print("\n---------- 闭合 / 吸合 ----------")
            for ch in grippers:
                print(f"{ch['name']} GripperClose: {client.GripperClose(ch['side'], timeout_ms=CMD_TIMEOUT_MS)}")
            for ch in suctions:
                print(f"{ch['name']} SuctionHold: {client.SuctionHold(ch['side'], timeout_ms=CMD_TIMEOUT_MS)}")
            time.sleep(0.2)
            for ch in grippers:
                print_gripper_status(client, ch, "闭合后")
            for ch in suctions:
                print_suction_status(client, ch, "吸合后")
            time.sleep(1.8)
    except KeyboardInterrupt:
        print("\n用户中断，退出。")


if __name__ == "__main__":
    main()
