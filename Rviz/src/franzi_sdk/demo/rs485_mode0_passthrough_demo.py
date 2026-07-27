#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple mode=0 RS485 passthrough menu demo.

This demo only uses ArmClient.Rs485Write and ArmClient.Rs485Read. Before running
it, make sure the target TIO channel is configured as RS485 multiplex mode=0.
"""
from __future__ import annotations

import os
import sys
import time
from typing import List, Sequence

_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core.arm_client import ArmClient
from core.types import ArmSide, SdkStatus


DEFAULT_HOST = "192.168.23.30"
DEFAULT_PORT = 8000
DEFAULT_TIMEOUT_MS = 1000
DEFAULT_HOLD_MS = 3


def parse_hex_payload(text: str) -> List[int]:
    raw = text.strip().replace(",", " ").replace("0x", "").replace("0X", "")
    if not raw:
        return []

    parts = raw.split()
    if len(parts) == 1 and len(parts[0]) > 2:
        token = parts[0]
        if len(token) % 2 != 0:
            raise ValueError("continuous hex string length must be even")
        parts = [token[i:i + 2] for i in range(0, len(token), 2)]

    payload = [int(part, 16) for part in parts]
    for byte in payload:
        if byte < 0 or byte > 0xFF:
            raise ValueError("byte value out of range 00..FF")
    return payload


def format_hex(payload: Sequence[int]) -> str:
    return " ".join(f"{int(byte) & 0xFF:02X}" for byte in payload)


def ask_int(prompt: str, default: int) -> int:
    text = input(f"{prompt} [{default}]: ").strip()
    return default if not text else int(text, 0)


def ask_side() -> ArmSide:
    while True:
        text = input("选择臂侧: l=左臂, r=右臂 [l]: ").strip().lower()
        if text in ("", "l", "left", "0"):
            return ArmSide.LEFT
        if text in ("r", "right", "1"):
            return ArmSide.RIGHT
        print("输入无效，请输入 l 或 r")


class Rs485MenuDemo:
    def __init__(self, arm: ArmClient, side: ArmSide):
        self.arm = arm
        self.side = side
        self.frame_id = 1
        self.timeout_ms = DEFAULT_TIMEOUT_MS
        self.hold_ms = DEFAULT_HOLD_MS

    def run(self) -> None:
        while True:
            self.print_menu()
            choice = input("请选择: ").strip().lower()
            try:
                if choice == "1":
                    self.send_once(wait_reply=True)
                elif choice == "2":
                    self.send_once(wait_reply=False)
                elif choice == "3":
                    self.read_once()
                elif choice == "4":
                    self.drain_replies()
                elif choice == "5":
                    self.configure()
                elif choice in ("q", "quit", "exit"):
                    return
                else:
                    print("未知选项")
            except Exception as ex:
                print(f"[ERROR] {type(ex).__name__}: {ex}")

    def print_menu(self) -> None:
        target = f"side={self.side.name}"
        print("\n========== RS485 mode=0 passthrough ==========")
        print(f"当前目标: {target}, timeout_ms={self.timeout_ms}, hold_ms={self.hold_ms}")
        print("1. 发送 HEX,并等待/读取回包")
        print("2. 只发送 HEX,不等待回包")
        print("3. 读取一次已有回包")
        print("4. 清空已有回包")
        print("5. 修改 side/timeout")
        print("q. 退出")

    def send_once(self, wait_reply: bool) -> None:
        text = input("输入发送 HEX,例如: 01 03 00 00 00 01 84 0A: ")
        payload = parse_hex_payload(text)
        frame_id = self.next_frame_id()
        status = self.arm.Rs485Write(
            self.side,
            payload,
            frame_id=frame_id,
            timeout_ms=self.timeout_ms,
            hold_ms=self.hold_ms,
            expect_reply=wait_reply,
        )
        self.check_status("Rs485Write", status)
        print(f"TX frame_id={frame_id}: {format_hex(payload)}")
        if wait_reply:
            self.wait_reply(frame_id)

    def wait_reply(self, frame_id: int) -> None:
        deadline = time.monotonic() + self.timeout_ms / 1000.0
        while time.monotonic() < deadline:
            rsp = self.arm.Rs485Read(self.side, consume=True)
            self.check_status("Rs485Read", int(rsp.get("status", SdkStatus.OK)))
            if rsp.get("has_data"):
                rx_frame_id = int(rsp.get("frame_id", 0))
                payload = [int(x) & 0xFF for x in rsp.get("payload", [])]
                prefix = "RX"
                if rx_frame_id != frame_id:
                    prefix = "RX(非本次 frame_id)"
                print(f"{prefix} frame_id={rx_frame_id}: {format_hex(payload)}")
                return
            time.sleep(0.01)
        print(f"等待回包超时: frame_id={frame_id}")

    def read_once(self) -> None:
        rsp = self.arm.Rs485Read(self.side, consume=True)
        self.check_status("Rs485Read", int(rsp.get("status", SdkStatus.OK)))
        if not rsp.get("has_data"):
            print("当前没有回包")
            return
        payload = [int(x) & 0xFF for x in rsp.get("payload", [])]
        print(f"RX frame_id={int(rsp.get('frame_id', 0))}: {format_hex(payload)}")

    def drain_replies(self) -> None:
        count = 0
        while True:
            rsp = self.arm.Rs485Read(self.side, consume=True)
            self.check_status("Rs485Read", int(rsp.get("status", SdkStatus.OK)))
            if not rsp.get("has_data"):
                break
            count += 1
            payload = [int(x) & 0xFF for x in rsp.get("payload", [])]
            print(f"丢弃 RX frame_id={int(rsp.get('frame_id', 0))}: {format_hex(payload)}")
        print(f"已清空 {count} 条回包")

    def configure(self) -> None:
        self.side = ask_side()
        self.timeout_ms = ask_int("timeout_ms", self.timeout_ms)
        self.hold_ms = ask_int("hold_ms", self.hold_ms)

    def next_frame_id(self) -> int:
        frame_id = self.frame_id
        self.frame_id = 1 if self.frame_id >= 0xFFFFFFFF else self.frame_id + 1
        return frame_id

    @staticmethod
    def check_status(where: str, status: int) -> None:
        if int(status) != int(SdkStatus.OK):
            raise RuntimeError(f"{where} failed: status={status}. 请确认 TIO 通道已配置为 mode=0")


def main() -> None:
    host = input(f"SdkServer IP [{DEFAULT_HOST}]: ").strip() or DEFAULT_HOST
    port = ask_int("SdkServer port", DEFAULT_PORT)
    side = ask_side()

    arm = ArmClient(host, port, raise_on_error=False)
    try:
        Rs485MenuDemo(arm, side).run()
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
