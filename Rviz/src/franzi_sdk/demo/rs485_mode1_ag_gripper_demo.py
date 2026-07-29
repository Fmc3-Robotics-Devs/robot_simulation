#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TIO mode=1 大寰 AG 夹爪 SDK 封装接口 Demo。

这个示例演示“客户直接使用 SDK 内置夹爪接口”的推荐写法：
1. 机器人侧 485 复用模块负责大寰 AG 夹爪协议适配。
2. 客户代码只需要调用 arm.AgInitialize/AgSetForce/AgSetPosition/
   AgReadPosition/AgReadGripState 等接口，不需要关心 Modbus RTU 帧、CRC 和寄存器。
3. 和 mode=0 透传示例相比，本示例更适合已经由 SDK 官方支持的 AG 夹爪。

运行前准备：
- 确认对应臂侧的 TIO 通道在 485 复用模块中配置为 mode=1。
- 当前服务端约定 ArmSide.LEFT -> TIO0，ArmSide.RIGHT -> TIO1。
- 确认夹爪 RS485 A/B 接线、供电、波特率、从站 ID 与配置一致。本示例默认从站 ID=1。

客户二次开发指引：
- 单夹爪测试时，将 TEST_MODE 改为 "MODE_SINGLE_GRIP"。
- 修改 grip*_cfg 中的 side/slave_id/name，即可适配不同臂侧或不同从站 ID。
- 常用控制接口：
    AgInitialize 初始化/找零；
    AgSetForce 设置夹持力；
    AgSetPosition 设置开合位置，0=闭合，1000=打开；
    AgReadPosition/AgReadForce/AgReadGripState/AgReadInitState 读取状态。
- 若接口返回非 0，请优先检查 TIO mode 是否为 1，以及夹爪从站 ID 是否正确。

运行：
  python3 sdk/sdk_py/demo/rs485_mode1_ag_gripper_demo.py
"""

import time
from core.arm_client import ArmClient
from core.types import ArmSide

# ==============================================================================
# 测试模式选择宏定义（注释切换模式，二选一）
# MODE_SINGLE_GRIP  = 单夹爪测试（左臂末端夹爪）
# MODE_DUAL_GRIP    = 双夹爪同步测试（左右臂末端夹爪）
# ==============================================================================
# TEST_MODE = "MODE_SINGLE_GRIP"
TEST_MODE = "MODE_DUAL_GRIP"
# ==============================================================================

# 机械臂通信全局配置
ARM_IP = "192.168.23.30"
ARM_PORT = 8000
CMD_TIMEOUT_MS = 1000

# 夹爪通用参数
OPEN_POS = 1000
CLOSE_POS = 0
FORCE = 40

# 夹爪配置字典
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

# 根据模式加载需要测试的夹爪列表
if TEST_MODE == "MODE_SINGLE_GRIP":
    all_grips = [grip1_cfg]
    print("【当前模式】单夹爪测试 MODE_SINGLE_GRIP")
elif TEST_MODE == "MODE_DUAL_GRIP":
    all_grips = [grip1_cfg, grip2_cfg]
    print("【当前模式】双夹爪同步测试 MODE_DUAL_GRIP")
else:
    raise RuntimeError("TEST_MODE 配置错误，请选择 MODE_SINGLE_GRIP / MODE_DUAL_GRIP")

# 创建客户端
arm = ArmClient(ARM_IP, ARM_PORT, raise_on_error=False)


def print_grip_status(grip, tag: str):
    """打印指定夹爪实时状态"""
    name = grip["name"]
    side = grip["side"]
    sid = grip["slave_id"]
    print(f"\n===== [{name}] {tag} =====")
    pos = arm.AgReadPosition(side, rs485_slave_id=sid)
    force = arm.AgReadForce(side, rs485_slave_id=sid)
    grip_state = arm.AgReadGripState(side, rs485_slave_id=sid)
    print(f"位置 pos: {pos}")
    print(f"实时力 force: {force}")
    print(f"夹持状态 grip: {grip_state}")


def single_grip_init_wait(grip, max_loop=100, sleep_dt=0.1):
    """单路夹爪初始化并等待就绪，超时直接退出程序"""
    name = grip["name"]
    side = grip["side"]
    sid = grip["slave_id"]
    force_val = grip["force"]

    # 执行初始化
    init_rsp = arm.AgInitialize(side, rs485_slave_id=sid, full_calibration=False)
    print(f"\n[{name}] 初始化指令返回: {init_rsp}")
    time.sleep(0.2)

    # 设置夹持力
    force_rsp = arm.AgSetForce(
        side, force_val,
        rs485_slave_id=sid,
        timeout_ms=CMD_TIMEOUT_MS
    )
    print(f"[{name}] 设置夹持力{force_val}返回: {force_rsp}")
    time.sleep(0.2)

    # 轮询等待初始化完成
    ready = False
    for i in range(max_loop):
        rsp = arm.AgReadInitState(side, rs485_slave_id=sid)
        print(f"[{name}] 初始化状态轮询{i}: {rsp}")
        if rsp.get("status") == 0 and rsp.get("value") == 1:
            ready = True
            break
        time.sleep(sleep_dt)

    if not ready:
        print(f"\n[{name}] 初始化等待超时，程序退出！")
        exit(1)
    print(f"[{name}] 初始化完成就绪")


# 初始化所有选中的夹爪
print("\n========== 开始初始化夹爪 ==========")
for grip in all_grips:
    single_grip_init_wait(grip)
time.sleep(0.5)
print("========== 夹爪全部就绪，进入循环测试 ==========")

try:
    while True:
        # 同步打开所有夹爪
        print("\n---------- 执行夹爪打开 ----------")
        for grip in all_grips:
            rsp = arm.AgSetPosition(
                grip["side"], grip["open_pos"],
                rs485_slave_id=grip["slave_id"],
                timeout_ms=CMD_TIMEOUT_MS
            )
            print(f"{grip['name']} open指令返回: {rsp}")

        time.sleep(0.1)
        for grip in all_grips:
            print_grip_status(grip, "打开后100ms状态")
        time.sleep(1.9)

        # 同步闭合所有夹爪
        print("\n---------- 执行夹爪闭合 ----------")
        for grip in all_grips:
            rsp = arm.AgSetPosition(
                grip["side"], grip["close_pos"],
                rs485_slave_id=grip["slave_id"],
                timeout_ms=CMD_TIMEOUT_MS
            )
            print(f"{grip['name']} close指令返回: {rsp}")

        time.sleep(0.1)
        for grip in all_grips:
            print_grip_status(grip, "闭合后100ms状态")
        time.sleep(1.9)

except KeyboardInterrupt:
    print("\n\n检测到Ctrl+C，终止夹爪循环测试！")
