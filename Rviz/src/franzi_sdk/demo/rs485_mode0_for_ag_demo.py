#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
客户侧 mode=0 原始 RS485 透传适配大寰 AG 夹爪 Demo。

这个示例演示“客户自主适配第三方 RS485 设备”的推荐写法：
1. 机器人侧只提供 mode=0 原始 RS485 透传能力，业务协议由客户自己实现。
2. 本文件在客户侧实现大寰 AG 夹爪的 Modbus RTU 协议，包括 CRC16、读保持寄存器
   0x03、写单寄存器 0x06、回包校验和寄存器语义。
3. 代码中不会调用 SDK 的 AgInitialize/AgSetPosition/AgReadPosition 等 mode=1
   夹爪封装接口，只调用 arm.Rs485Write(...) 和 arm.Rs485Read(...)。

运行前准备：
- 确认对应臂侧的 TIO 通道在 485 复用模块中配置为 mode=0。
- 当前服务端约定 ArmSide.LEFT -> TIO0，ArmSide.RIGHT -> TIO1。
- 确认夹爪 RS485 A/B 接线、供电、波特率、从站 ID 与配置一致。本示例默认从站 ID=1。

客户二次开发指引：
- 适配其它 RS485 设备时，保留 DahuanAgGripperRaw485.transact() 的收发框架，
  替换 build_* / parse_* 协议函数即可。
- 每次请求建议使用递增 frame_id，并在回包中校验 frame_id，避免读到旧回包。
- 发送新请求前建议先清理旧回包，本示例 _drain_stale_reply() 已做这个处理。
- payload 最大长度受 485 复用模块限制，当前服务端限制为 16 字节。
- 若 Rs485Write/Rs485Read 返回非 0，请优先检查 TIO mode 是否为 0。

运行：
  python3 sdk/sdk_py/demo/rs485_mode0_for_ag_demo.py
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


# ==============================================================================
# 测试模式选择
# ==============================================================================
# TEST_MODE = "MODE_SINGLE_GRIP"
TEST_MODE = "MODE_DUAL_GRIP"

# 机械臂通信全局配置
ARM_IP = "192.168.23.30"
ARM_PORT = 8000
CMD_TIMEOUT_MS = 1000

# 夹爪通用参数
OPEN_POS = 1000
CLOSE_POS = 0
FORCE = 40

# 大寰 AG 夹爪 Modbus RTU 寄存器
FUNC_READ_HOLDING = 0x03
FUNC_WRITE_SINGLE = 0x06
REG_INIT = 0x0100
REG_FORCE = 0x0101
REG_POSITION_SET = 0x0103
REG_INIT_STATE = 0x0200
REG_GRIP_STATE = 0x0201
REG_POSITION_FEEDBACK = 0x0202
INIT_FIND_ZERO = 0x0001
INIT_FULL_CALIBRATION = 0x00A5


grip1_cfg = {
    "side": ArmSide.LEFT,
    "slave_id": 1,
    "open_pos": OPEN_POS,
    "close_pos": CLOSE_POS,
    "force": FORCE,
    "name": "左臂夹爪",
}
grip2_cfg = {
    "side": ArmSide.RIGHT,
    "slave_id": 1,
    "open_pos": OPEN_POS,
    "close_pos": CLOSE_POS,
    "force": FORCE,
    "name": "右臂夹爪",
}

if TEST_MODE == "MODE_SINGLE_GRIP":
    all_grips = [grip1_cfg]
    print("【当前模式】单夹爪测试 MODE_SINGLE_GRIP")
elif TEST_MODE == "MODE_DUAL_GRIP":
    all_grips = [grip1_cfg, grip2_cfg]
    print("【当前模式】双夹爪同步测试 MODE_DUAL_GRIP")
else:
    raise RuntimeError("TEST_MODE 配置错误，请选择 MODE_SINGLE_GRIP / MODE_DUAL_GRIP")


def crc16_modbus(data: Sequence[int]) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= int(byte) & 0xFF
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def put_u16_be(value: int) -> List[int]:
    return [(int(value) >> 8) & 0xFF, int(value) & 0xFF]


def get_u16_be(data: Sequence[int], offset: int) -> int:
    return ((int(data[offset]) & 0xFF) << 8) | (int(data[offset + 1]) & 0xFF)


def append_crc(frame_without_crc: Sequence[int]) -> List[int]:
    frame = [int(x) & 0xFF for x in frame_without_crc]
    crc = crc16_modbus(frame)
    frame.extend([crc & 0xFF, (crc >> 8) & 0xFF])
    return frame


def build_write_single(slave_id: int, reg: int, value: int) -> List[int]:
    return append_crc([
        slave_id,
        FUNC_WRITE_SINGLE,
        *put_u16_be(reg),
        *put_u16_be(value),
    ])


def build_read_holding(slave_id: int, reg: int, count: int = 1) -> List[int]:
    return append_crc([
        slave_id,
        FUNC_READ_HOLDING,
        *put_u16_be(reg),
        *put_u16_be(count),
    ])


def check_crc(frame: Sequence[int]) -> bool:
    if len(frame) < 4:
        return False
    expected = crc16_modbus(frame[:-2])
    actual = (int(frame[-1]) << 8) | int(frame[-2])
    return expected == actual


def hex_frame(frame: Sequence[int]) -> str:
    return " ".join(f"{int(x) & 0xFF:02X}" for x in frame)


def parse_write_echo(frame: Sequence[int], slave_id: int, reg: int, value: int) -> None:
    if len(frame) != 8 or not check_crc(frame):
        raise RuntimeError(f"写寄存器回包格式错误: {hex_frame(frame)}")
    if int(frame[1]) == (FUNC_WRITE_SINGLE | 0x80):
        raise RuntimeError(f"写寄存器异常回包: {hex_frame(frame)}")
    echo_slave = int(frame[0])
    echo_func = int(frame[1])
    echo_reg = get_u16_be(frame, 2)
    echo_value = get_u16_be(frame, 4)
    if echo_slave != slave_id or echo_func != FUNC_WRITE_SINGLE or echo_reg != reg or echo_value != value:
        raise RuntimeError(f"写寄存器回包不匹配: {hex_frame(frame)}")


def parse_read_reply(frame: Sequence[int], slave_id: int) -> List[int]:
    if len(frame) < 7 or not check_crc(frame):
        raise RuntimeError(f"读寄存器回包格式错误: {hex_frame(frame)}")
    if int(frame[1]) == (FUNC_READ_HOLDING | 0x80):
        raise RuntimeError(f"读寄存器异常回包: {hex_frame(frame)}")
    if int(frame[0]) != slave_id or int(frame[1]) != FUNC_READ_HOLDING:
        raise RuntimeError(f"读寄存器回包不匹配: {hex_frame(frame)}")

    byte_count = int(frame[2])
    if byte_count % 2 != 0 or len(frame) != byte_count + 5:
        raise RuntimeError(f"读寄存器字节数错误: {hex_frame(frame)}")
    return [get_u16_be(frame, 3 + i * 2) for i in range(byte_count // 2)]


class DahuanAgGripperRaw485:
    """客户侧大寰 AG 夹爪适配层，只依赖 ArmClient.Rs485Write/Rs485Read。"""

    def __init__(self, arm: ArmClient, side: ArmSide, slave_id: int = 1, timeout_ms: int = 500):
        self.arm = arm
        self.side = side
        self.slave_id = int(slave_id)
        self.timeout_ms = int(timeout_ms)
        self._frame_id = 1

    def initialize(self, full_calibration: bool = False) -> None:
        value = INIT_FULL_CALIBRATION if full_calibration else INIT_FIND_ZERO
        self.write_register(REG_INIT, value)

    def set_position(self, position: int) -> None:
        if not 0 <= int(position) <= 1000:
            raise ValueError("position must be 0..1000")
        self.write_register(REG_POSITION_SET, int(position))

    def set_force(self, force: int) -> None:
        if not 20 <= int(force) <= 100:
            raise ValueError("force must be 20..100")
        self.write_register(REG_FORCE, int(force))

    def open(self) -> None:
        self.set_position(OPEN_POS)

    def close(self) -> None:
        self.set_position(CLOSE_POS)

    def read_position(self) -> int:
        return self.read_register(REG_POSITION_FEEDBACK)

    def read_force(self) -> int:
        return self.read_register(REG_FORCE)

    def read_grip_state(self) -> int:
        return self.read_register(REG_GRIP_STATE)

    def read_init_state(self) -> int:
        return self.read_register(REG_INIT_STATE)

    def write_register(self, reg: int, value: int) -> None:
        request = build_write_single(self.slave_id, reg, value)
        reply = self.transact(request)
        parse_write_echo(reply, self.slave_id, reg, value)

    def read_register(self, reg: int) -> int:
        request = build_read_holding(self.slave_id, reg, 1)
        reply = self.transact(request)
        values = parse_read_reply(reply, self.slave_id)
        if not values:
            raise RuntimeError(f"读寄存器无数据: reg=0x{reg:04X}")
        return values[0]

    def transact(self, request: Sequence[int]) -> List[int]:
        self._drain_stale_reply()
        frame_id = self._next_frame_id()
        status = self.arm.Rs485Write(
            self.side,
            request,
            frame_id=frame_id,
            timeout_ms=self.timeout_ms,
            hold_ms=3,
            expect_reply=True,
        )
        self._check_status("Rs485Write", status)

        deadline = time.monotonic() + self.timeout_ms / 1000.0
        while time.monotonic() < deadline:
            rsp = self.arm.Rs485Read(self.side, consume=True)
            self._check_status("Rs485Read", int(rsp.get("status", SdkStatus.OK)))
            if rsp.get("has_data"):
                if int(rsp.get("frame_id", 0)) != frame_id:
                    continue
                return [int(x) & 0xFF for x in rsp.get("payload", [])]
            time.sleep(0.01)
        raise TimeoutError(f"等待 RS485 回包超时: frame_id={frame_id}, tx={hex_frame(request)}")

    def _drain_stale_reply(self) -> None:
        for _ in range(4):
            rsp = self.arm.Rs485Read(self.side, consume=True)
            self._check_status("Rs485Read", int(rsp.get("status", SdkStatus.OK)))
            if not rsp.get("has_data"):
                return
            print(f"[WARN] 丢弃旧 RS485 回包: {hex_frame(rsp.get('payload', []))}")

    def _next_frame_id(self) -> int:
        frame_id = self._frame_id
        self._frame_id = 1 if self._frame_id >= 0xFFFFFFFF else self._frame_id + 1
        return frame_id

    @staticmethod
    def _check_status(where: str, status: int) -> None:
        if int(status) != int(SdkStatus.OK):
            raise RuntimeError(
                f"{where} failed: status={status}. "
                "请确认对应 TIO 通道已配置为 mode=0，并且夹爪接线/从站 ID 正确。"
            )


def print_grip_status(raw_grip: DahuanAgGripperRaw485, name: str, tag: str) -> None:
    print(f"\n===== [{name}] {tag} =====")
    print(f"位置 pos: {raw_grip.read_position()}")
    print(f"实时力 force: {raw_grip.read_force()}")
    print(f"夹持状态 grip: {raw_grip.read_grip_state()}")


def single_grip_init_wait(raw_grip: DahuanAgGripperRaw485, name: str, force: int,
                          max_loop: int = 100, sleep_dt: float = 0.1) -> None:
    raw_grip.initialize(full_calibration=False)
    print(f"\n[{name}] 初始化指令已发送")
    time.sleep(0.2)

    raw_grip.set_force(force)
    print(f"[{name}] 设置夹持力{force}完成")
    time.sleep(0.2)

    for i in range(max_loop):
        init_state = raw_grip.read_init_state()
        print(f"[{name}] 初始化状态轮询{i}: value={init_state}")
        if init_state == 1:
            print(f"[{name}] 初始化完成就绪")
            return
        time.sleep(sleep_dt)

    raise TimeoutError(f"[{name}] 初始化等待超时")


def main() -> None:
    arm = ArmClient(ARM_IP, ARM_PORT, raise_on_error=False)
    raw_grips = [
        {
            **cfg,
            "driver": DahuanAgGripperRaw485(
                arm,
                side=cfg["side"],
                slave_id=cfg["slave_id"],
                timeout_ms=CMD_TIMEOUT_MS,
            ),
        }
        for cfg in all_grips
    ]

    try:
        print("\n========== 开始初始化夹爪（mode=0 raw RS485）==========")
        for grip in raw_grips:
            single_grip_init_wait(grip["driver"], grip["name"], grip["force"])
        time.sleep(0.5)
        print("========== 夹爪全部就绪，进入循环测试 ==========")

        while True:
            print("\n---------- 执行夹爪打开 ----------")
            for grip in raw_grips:
                grip["driver"].set_position(grip["open_pos"])
                print(f"{grip['name']} open 指令完成")
            time.sleep(0.1)
            for grip in raw_grips:
                print_grip_status(grip["driver"], grip["name"], "打开后100ms状态")
            time.sleep(1.9)

            print("\n---------- 执行夹爪闭合 ----------")
            for grip in raw_grips:
                grip["driver"].set_position(grip["close_pos"])
                print(f"{grip['name']} close 指令完成")
            time.sleep(0.1)
            for grip in raw_grips:
                print_grip_status(grip["driver"], grip["name"], "闭合后100ms状态")
            time.sleep(1.9)

    except KeyboardInterrupt:
        print("\n\n检测到 Ctrl+C，终止夹爪循环测试！")
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
