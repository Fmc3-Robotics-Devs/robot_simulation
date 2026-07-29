#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VR 双臂遥操作 — 经 ArmClient RPC 推送笛卡尔位姿。"""


from __future__ import annotations

import os
import signal
import sys
import threading
import time
from threading import Event, Thread

import numpy as np
import scipy.spatial.transform as st

from core.client import ArmClient
from core.types import MechUnitLifecycleCmd, MechUnitType
from common.utils import add_pose_increment, convert_to_rotation, precise_wait

INBC_FOR_HUMANOID_HOST = "192.168.23.30"
INBC_FOR_HUMANOID_PORT = 8000

# 双臂遥操作控制频率（Hz），修改此值即可调整控制周期
CONTROL_FREQUENCY_HZ = 50
CONTROL_INTERVAL_SEC = 1.0 / CONTROL_FREQUENCY_HZ

# 遥操作臂配置: "left" | "right" | "both"
TELEOP_ARM_MODE = "both"

_VALID_TELEOP_ARM_MODES = frozenset({"left", "right", "both"})
_TELEOP_ARM_MODE_LABELS = {"left": "左臂", "right": "右臂", "both": "双臂"}

# 夹爪控制配置
GRIPPER_CONTROL_ENABLED = True          # 是否启用夹爪遥操作
GRIPPER_MODE = "both"                   # 夹爪使能模式: "left" | "right" | "both"
GRIPPER_TRIGGER_THRESHOLD = 0.05        # 扳机有效阈值（小于此值视为0，避免抖动）
GRIPPER_POSITION_MIN = 0                # 夹爪闭合位置
GRIPPER_POSITION_MAX = 1000             # 夹爪张开位置
GRIPPER_CMD_TIMEOUT_MS = 500            # 夹爪指令超时（毫秒）

_VALID_GRIPPER_MODES = frozenset({"left", "right", "both"})
_GRIPPER_MODE_LABELS = {"left": "左夹爪", "right": "右夹爪", "both": "双夹爪"}


def _resolve_gripper_mode(mode: str) -> tuple[bool, bool, str]:
    if mode not in _VALID_GRIPPER_MODES:
        raise ValueError(
            f"GRIPPER_MODE 无效: {mode!r}，可选: {sorted(_VALID_GRIPPER_MODES)}"
        )
    enable_left = mode in ("left", "both")
    enable_right = mode in ("right", "both")
    return enable_left, enable_right, _GRIPPER_MODE_LABELS[mode]


GRIPPER_LEFT_ENABLED, GRIPPER_RIGHT_ENABLED, GRIPPER_MODE_LABEL = _resolve_gripper_mode(
    GRIPPER_MODE
)


def _resolve_teleop_arm_mode(mode: str) -> tuple[bool, bool, str]:
    if mode not in _VALID_TELEOP_ARM_MODES:
        raise ValueError(
            f"TELEOP_ARM_MODE 无效: {mode!r}，可选: {sorted(_VALID_TELEOP_ARM_MODES)}"
        )
    enable_left = mode in ("left", "both")
    enable_right = mode in ("right", "both")
    return enable_left, enable_right, _TELEOP_ARM_MODE_LABELS[mode]


TELEOP_LEFT_ARM_ENABLED, TELEOP_RIGHT_ARM_ENABLED, TELEOP_ARM_MODE_LABEL = _resolve_teleop_arm_mode(
    TELEOP_ARM_MODE
)

np.set_printoptions(suppress=True, precision=4)


class MultiTopicPublisher:
    """VR 双臂笛卡尔遥操作控制器（RPC）。"""

    def __init__(self, quest=None, domain_id=None, image_domain_id=None):
        del domain_id, image_domain_id  # 兼容旧入口，已不再使用 DDS

        self.quest = quest
        self.base_left_arm_pose = np.zeros(6)
        self.base_right_arm_pose = np.zeros(6)
        self.base_left_arm_pose_deg = np.zeros(6)
        self.base_right_arm_pose_deg = np.zeros(6)

        self.shutdown_event = Event()
        self.state_lock = threading.Lock()
        self.latest_left_pose: np.ndarray | None = None
        self.latest_right_pose: np.ndarray | None = None

        self.control_active = False
        self.control_started = False
        self.new_state_available = False
        self.control_interval = CONTROL_INTERVAL_SEC

        self.left_arm_update_teleop = 0
        self.right_arm_update_teleop = 0
        self._last_left_push_status = None
        self._last_right_push_status = None
        self.teleop_left_enabled = TELEOP_LEFT_ARM_ENABLED
        self.teleop_right_enabled = TELEOP_RIGHT_ARM_ENABLED
        self.teleop_arm_mode_label = TELEOP_ARM_MODE_LABEL

        # 夹爪控制状态
        self.gripper_control_enabled = GRIPPER_CONTROL_ENABLED
        self.gripper_left_enabled = GRIPPER_LEFT_ENABLED
        self.gripper_right_enabled = GRIPPER_RIGHT_ENABLED
        self.gripper_mode_label = GRIPPER_MODE_LABEL
        self._last_left_gripper_pos = None
        self._last_right_gripper_pos = None

        def signal_handler(signum, frame):
            print(f"\n接收到信号 {signum}，正在关闭...")
            self.shutdown_event.set()
            time.sleep(0.1)
            os._exit(0)

        signal.signal(signal.SIGINT, signal_handler)
        self.init_arm_client()

        self.control_thread = Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()

    def init_arm_client(self):
        print("\n初始化 ArmClient RPC ...")
        print(f"  遥操作配置: {self.teleop_arm_mode_label} (TELEOP_ARM_MODE={TELEOP_ARM_MODE!r})")
        self.arm_client = ArmClient(
            host=INBC_FOR_HUMANOID_HOST,
            port=INBC_FOR_HUMANOID_PORT,
            raise_on_error=False,
        )
        import math
        if self.teleop_left_enabled:
            ret = self.arm_client.SetMechUnitLifecycle(MechUnitType.LEFTARM, MechUnitLifecycleCmd.ENABLE)
    
            time.sleep(0.1)
            self.arm_client.MoveJ(
                joint_path=[[0, 0, 0, math.pi/2, 0, 0, 0]],
                side=0,
            )
            time.sleep(0.1)
        if self.teleop_right_enabled:
            ret = self.arm_client.SetMechUnitLifecycle(MechUnitType.RIGHTARM, MechUnitLifecycleCmd.ENABLE)
            time.sleep(0.1)
            self.arm_client.MoveJ(
                joint_path=[[0, 0, 0, math.pi/2, 0, 0, 0]],
                side=1,
            )
            time.sleep(0.1)
        if self.gripper_control_enabled:
            print(f"  夹爪遥操作: {self.gripper_mode_label} (GRIPPER_MODE={GRIPPER_MODE!r})")
            self._init_grippers()

    def _init_grippers(self):
        """初始化夹爪（可选，若 yaml 中 auto_initialize=true 则可能已完成）。"""
        try:
            if self.gripper_left_enabled:
                print("\n========== [左夹爪] 初始化 ==========")
                self.arm_client.GripperInitialize(0, full_calibration=False, timeout_ms=GRIPPER_CMD_TIMEOUT_MS)
                time.sleep(0.2)
                for i in range(100):
                    rsp = self.arm_client.GripperReadInitState(0, timeout_ms=GRIPPER_CMD_TIMEOUT_MS)
                    if rsp.get("status") == 0 and rsp.get("value") == 1:
                        print("[左夹爪] 初始化完成")
                        break
                    time.sleep(0.1)
                else:
                    print("[左夹爪] 警告：初始化轮询超时")

            if self.gripper_right_enabled:
                print("\n========== [右夹爪] 初始化 ==========")
                self.arm_client.GripperInitialize(1, full_calibration=False, timeout_ms=GRIPPER_CMD_TIMEOUT_MS)
                time.sleep(0.2)
                for i in range(100):
                    rsp = self.arm_client.GripperReadInitState(1, timeout_ms=GRIPPER_CMD_TIMEOUT_MS)
                    if rsp.get("status") == 0 and rsp.get("value") == 1:
                        print("[右夹爪] 初始化完成")
                        break
                    time.sleep(0.1)
                else:
                    print("[右夹爪] 警告：初始化轮询超时")
        except Exception as e:
            print(f"[夹爪初始化] 异常: {e}")
            import traceback
            traceback.print_exc()

    @staticmethod
    def rad_to_deg(pose_rad: np.ndarray) -> np.ndarray:
        pose_deg = pose_rad.copy()
        pose_deg[3:] = np.degrees(pose_rad[3:])
        return pose_deg

    @staticmethod
    def deg_to_rad(pose_deg: np.ndarray) -> np.ndarray:
        pose_rad = pose_deg.copy()
        pose_rad[3:] = np.radians(pose_deg[3:])
        return pose_rad

    def _reset_vr_ref_poses(self):
        """清零 VR 参考位姿，防止再次按 X 时增量相对旧参考计算导致突变。"""
        if not self.quest:
            return
        keys = ["h"]
        if self.teleop_left_enabled:
            keys.extend(["l", "L"])
        if self.teleop_right_enabled:
            keys.extend(["r", "R"])
        for key in keys:
            self.quest.ref_pose[key] = None

    def _reset_gripper_state(self):
        """重置夹爪状态缓存。"""
        self._last_left_gripper_pos = None
        self._last_right_gripper_pos = None

    def set_base_poses_from_robot(self, left_pose=None, right_pose=None):
        print("\n" + "=" * 60)
        print(f"{self.teleop_arm_mode_label}基准位姿已更新")

        if self.teleop_left_enabled and left_pose is not None:
            self.base_left_arm_pose = np.array(left_pose, dtype=float)
            self.base_left_arm_pose_deg = self.rad_to_deg(self.base_left_arm_pose)
            with self.state_lock:
                self.latest_left_pose = self.base_left_arm_pose.copy()
            print(f"  左臂（弧度）: {self.base_left_arm_pose}")
            print(f"  左臂（度）  : {self.base_left_arm_pose_deg}")

        if self.teleop_right_enabled and right_pose is not None:
            self.base_right_arm_pose = np.array(right_pose, dtype=float)
            self.base_right_arm_pose_deg = self.rad_to_deg(self.base_right_arm_pose)
            with self.state_lock:
                self.latest_right_pose = self.base_right_arm_pose.copy()
            print(f"  右臂（弧度）: {self.base_right_arm_pose}")
            print(f"  右臂（度）  : {self.base_right_arm_pose_deg}")

        print("=" * 60)

    def fetch_robot_poses(self) -> bool:
        try:
            left_pose = right_pose = None

            if self.teleop_left_enabled:
                left_status, left_pose = self.arm_client.GetRobotPose(side=0)
                if left_status != 0:
                    print(f"[RPC] GetRobotPose LEFT 失败, status={left_status}")
                    return False

            if self.teleop_right_enabled:
                right_status, right_pose = self.arm_client.GetRobotPose(side=1)
                if right_status != 0:
                    print(f"[RPC] GetRobotPose RIGHT 失败, status={right_status}")
                    return False

            self.set_base_poses_from_robot(left_pose, right_pose)
            self.new_state_available = True
            #print(f"[RPC] GetRobotPose 成功: left={left_pose}, right={right_pose}")
            return True
        except Exception as e:
            print(f"[RPC] GetRobotPose 异常: {e}")
            import traceback
            traceback.print_exc()
        return False

    def start_teleop(self) -> bool:
        try:
            if self.teleop_left_enabled:
                left_ret = self.arm_client.TeleCartesian(side=0)
                if left_ret != 0:
                    print(f"[RPC] TeleCartesian LEFT 失败, status={left_ret}")
                    return False

            if self.teleop_right_enabled:
                right_ret = self.arm_client.TeleCartesian(side=1)
                if right_ret != 0:
                    print(f"[RPC] TeleCartesian RIGHT 失败, status={right_ret}")
                    return False

            print(f"[RPC] TeleCartesian {self.teleop_arm_mode_label}启动成功")
            return True
        except Exception as e:
            print(f"[RPC] TeleCartesian 异常: {e}")
            import traceback
            traceback.print_exc()
        return False
    def stop_teleop(self) -> bool:
        try:
            if self.teleop_left_enabled:
                left_ret = self.arm_client.StopTeleCartesian(side=0)
                if left_ret != 0:
                    print(f"[RPC] StopTeleCartesian LEFT 失败, status={left_ret}")
                    return False

            if self.teleop_right_enabled:
                right_ret = self.arm_client.StopTeleCartesian(side=1)
                if right_ret != 0:
                    print(f"[RPC] StopTeleCartesian RIGHT 失败, status={right_ret}")
                    return False

            print(f"[RPC] StopTeleCartesian {self.teleop_arm_mode_label}停止成功")
            return True
        except Exception as e:
            print(f"[RPC] StopTeleCartesian 异常: {e}")
            import traceback
            traceback.print_exc()
        return False
    def send_arm_command(self):
        if self.shutdown_event.is_set() or not self.quest:
            return

        # 如果 X 已经松开，直接跳过，避免竞争条件
        if not self.quest.is_x_pressed():
            return
        
        skip_rpc_push = False
        left_pose_rad = self.base_left_arm_pose.copy()
        right_pose_rad = self.base_right_arm_pose.copy()

        try:
            if self.teleop_left_enabled:
                left_arm_increment = self.quest.get_left_arm_world_increment()

                # if np.allclose(left_arm_increment, 0):
                #     skip_rpc_push = True
                # else:
                #     wk_left_arm_increment = convert_to_rotation(
                #         self.quest.get_left_arm_world_increment_R_no_need_pressed()
                #     )

                wk_left_arm_increment = convert_to_rotation(
                    self.quest.get_left_arm_world_increment_R_no_need_pressed()
                )

                left_arm_pose_deg = add_pose_increment(self.base_left_arm_pose_deg, left_arm_increment)
                drot = wk_left_arm_increment
                left_arm_pose_deg[3:] = (
                    drot * st.Rotation.from_euler("zyx", self.base_left_arm_pose_deg[3:], degrees=True)
                ).as_euler("zyx", degrees=True)
                left_pose_rad = self.deg_to_rad(left_arm_pose_deg)

            if self.teleop_right_enabled:
                right_arm_increment = self.quest.get_right_arm_world_increment()
                
                # if np.allclose(right_arm_increment, 0):
                #     skip_rpc_push = True
                # else:
                #     wk_right_arm_increment = convert_to_rotation(
                #         self.quest.get_right_arm_world_increment_R_no_need_pressed()
                #     )

                wk_right_arm_increment = convert_to_rotation(
                    self.quest.get_right_arm_world_increment_R_no_need_pressed()
                )

                right_arm_pose_deg = add_pose_increment(self.base_right_arm_pose_deg, right_arm_increment)
                drot2 = wk_right_arm_increment
                right_arm_pose_deg[3:] = (
                    drot2 * st.Rotation.from_euler("zyx", self.base_right_arm_pose_deg[3:], degrees=True)
                ).as_euler("zyx", degrees=True)
                right_pose_rad = self.deg_to_rad(right_arm_pose_deg)
        except Exception as e:
            print(f"获取 VR 增量失败: {e}")
            import traceback
            traceback.print_exc()
            with self.state_lock:
                left_ok = (not self.teleop_left_enabled) or self.latest_left_pose is not None
                right_ok = (not self.teleop_right_enabled) or self.latest_right_pose is not None
                if left_ok and right_ok:
                    if self.teleop_left_enabled and self.latest_left_pose is not None:
                        left_pose_rad = self.latest_left_pose.copy()
                    if self.teleop_right_enabled and self.latest_right_pose is not None:
                        right_pose_rad = self.latest_right_pose.copy()
                else:
                    skip_rpc_push = True

        if skip_rpc_push:
            print("[RPC] 跳过零位姿推送，等待下次有效 VR 数据")
            return

        try:
            t_rpc_start = time.monotonic()
            if self.teleop_left_enabled:
                print(f"[目标位姿] 左臂: {np.round(left_pose_rad, 4)}")##############
                push_left = self.arm_client.PushCartesianTeleopQueue(
                    side=0, target_pose=left_pose_rad.tolist()
                )
                if push_left != self._last_left_push_status and push_left != 0:
                    print(f"[RPC] PushCartesianTeleopQueue LEFT 失败, status={push_left}")
                self._last_left_push_status = push_left

            if self.teleop_right_enabled:
                print(f"[目标位姿] 右臂: {np.round(right_pose_rad, 4)}") ############
                push_right = self.arm_client.PushCartesianTeleopQueue(
                    side=1, target_pose=right_pose_rad.tolist()
                )
                if push_right != self._last_right_push_status and push_right != 0:
                    print(f"[RPC] PushCartesianTeleopQueue RIGHT 失败, status={push_right}")
                self._last_right_push_status = push_right

            self._rpc_push_cost = time.monotonic() - t_rpc_start
        except Exception as e:
            if not self.shutdown_event.is_set():
                print(f"推送{self.teleop_arm_mode_label}位姿失败: {e}")

    def _send_gripper_commands(self):
        """根据手柄扳机值发送夹爪位置指令。

        映射逻辑（反转）:
        - 不按扳机 (0) → 夹爪张开 (GRIPPER_POSITION_MAX=1000)
        - 按下扳机 (1) → 夹爪闭合 (GRIPPER_POSITION_MIN=0)
        """
        if not self.gripper_control_enabled or not self.quest:
            return

        try:
            if self.gripper_left_enabled:
                left_trigger = self.quest.get_left_gripper()
                if left_trigger < GRIPPER_TRIGGER_THRESHOLD:
                    left_trigger = 0.0
                # 反转映射: 按下扳机(1) → 闭合(0), 松开(0) → 张开(1000)
                left_gripper_pos = int((1.0 - left_trigger) * GRIPPER_POSITION_MAX)
                if self._last_left_gripper_pos != left_gripper_pos:
                    print(f"[夹爪调试] 左扳机={left_trigger:.3f} → 左夹爪位置={left_gripper_pos}")
                    self.arm_client.GripperSetPosition(
                        0, left_gripper_pos, timeout_ms=GRIPPER_CMD_TIMEOUT_MS
                    )
                    self._last_left_gripper_pos = left_gripper_pos

            if self.gripper_right_enabled:
                right_trigger = self.quest.get_right_gripper()
                if right_trigger < GRIPPER_TRIGGER_THRESHOLD:
                    right_trigger = 0.0
                # 反转映射: 按下扳机(1) → 闭合(0), 松开(0) → 张开(1000)
                right_gripper_pos = int((1.0 - right_trigger) * GRIPPER_POSITION_MAX)
                if self._last_right_gripper_pos != right_gripper_pos:
                    print(f"[夹爪调试] 右扳机={right_trigger:.3f} → 右夹爪位置={right_gripper_pos}")
                    self.arm_client.GripperSetPosition(
                        1, right_gripper_pos, timeout_ms=GRIPPER_CMD_TIMEOUT_MS
                    )
                    self._last_right_gripper_pos = right_gripper_pos
        except Exception as e:
            if not self.shutdown_event.is_set():
                print(f"[夹爪控制] 发送指令失败: {e}")

    def control_loop(self):
        print(f"VR {self.teleop_arm_mode_label}控制线程启动...")

        last_x_pressed = False
        last_button_check_time = 0.0
        button_check_interval = 0.02

        t_start = time.monotonic()
        iter_idx = 0
        last_loop_time = time.monotonic()

        while not self.shutdown_event.is_set():
            loop_entry_time = time.monotonic()
            current_time = time.time()
            current_monotonic = time.monotonic()

            loop_period = loop_entry_time - last_loop_time
            # if iter_idx % 50 == 0 and iter_idx > 0:
            #     print(f"[循环周期] iter={iter_idx}, 间隔={loop_period * 1000:.1f}ms ({1 / loop_period:.1f}Hz)")
            last_loop_time = loop_entry_time

            if self.quest:
                try:
                    self.quest.update()
                except Exception as e:
                    if iter_idx % 50 == 0:
                        print(f"VR 更新错误: {e}")

            if self.quest and (current_time - last_button_check_time >= button_check_interval):
                try:
                    current_x_pressed = self.quest.is_x_pressed()

                    if current_x_pressed and not last_x_pressed:
                        print("\n" + "=" * 60)
                        print(f"[VR 控制] X 按钮按下，开始{self.teleop_arm_mode_label}遥操作")

                        self.control_active = True
                        self.control_started = False
                        self.new_state_available = False
                        self.left_arm_update_teleop = 1
                        self.right_arm_update_teleop = 1

                        t_start = time.monotonic()
                        iter_idx = 0

                        self._reset_vr_ref_poses()
                        self._reset_gripper_state()
                        if self.fetch_robot_poses():
                            self.start_teleop()
                        else:
                            self.control_active = False

                        print("=" * 60)

                    elif not current_x_pressed and last_x_pressed:
                        print("\n" + "=" * 60)
                        print(f"[VR 控制] X 按钮松开，停止{self.teleop_arm_mode_label}遥操作")

                        self.control_active = False
                        self.control_started = False
                        self.new_state_available = False
                        self.left_arm_update_teleop = 0
                        self.right_arm_update_teleop = 0
                        
                        self.stop_teleop()

                        with self.state_lock:
                            if self.teleop_left_enabled:
                                self.latest_left_pose = None
                            if self.teleop_right_enabled:
                                self.latest_right_pose = None
                        self._reset_vr_ref_poses()
                        self._reset_gripper_state()
                        print("=" * 60)

                    last_x_pressed = current_x_pressed
                    last_button_check_time = current_time
                except Exception as e:
                    if iter_idx % 50 == 0:
                        print(f"获取 X 按钮状态错误: {e}")

            if self.control_active and not self.control_started:
                if self.new_state_available:
                    self.new_state_available = False
                    self.control_started = True
                    print(f"[VR 控制] 基准位姿就绪，开始发送{self.teleop_arm_mode_label}指令")

            if self.control_active and self.control_started:
                t_cycle_end = t_start + iter_idx * self.control_interval
                if current_monotonic >= t_cycle_end:
                    try:
                        loop_start = time.monotonic()
                        self.send_arm_command()
                        self._send_gripper_commands()
                        self.left_arm_update_teleop = 0
                        self.right_arm_update_teleop = 0

                        now_after_exec = time.monotonic()
                        t_cycle_end = t_start + (iter_idx + 1) * self.control_interval
                        if now_after_exec < t_cycle_end:
                            precise_wait(t_cycle_end)
                        else:
                            if iter_idx % 50 == 0:
                                overrun_ms = (now_after_exec - t_cycle_end) * 1000
                                print(f"控制循环超时 {overrun_ms:.0f}ms (iter={iter_idx})，校准基准")
                            t_start = now_after_exec
                            iter_idx = 0

                        delta_time = time.monotonic() - loop_start
                        # if iter_idx % 10 == 0 and delta_time > 1e-6:
                        #     print(f"[实际频率] {1 / delta_time:.2f}Hz (循环时间: {delta_time * 1000:.1f}ms)")

                        iter_idx += 1
                    except Exception as e:
                        print(f"发送控制命令错误: {e}")
                        import traceback
                        traceback.print_exc()
                        self.control_started = False
                else:
                    time.sleep(0.001)
            else:
                time.sleep(0.001)

        print("VR 双臂控制线程退出")

    def run(self):
        print("\n" + "=" * 60)
        print(f"VR {self.teleop_arm_mode_label}遥操作")
        print(f"  控制器: {INBC_FOR_HUMANOID_HOST}:{INBC_FOR_HUMANOID_PORT}")
        print(f"  遥操作臂: {self.teleop_arm_mode_label} (TELEOP_ARM_MODE={TELEOP_ARM_MODE!r})")
        print(f"  控制频率: {CONTROL_FREQUENCY_HZ} Hz (周期 {CONTROL_INTERVAL_SEC * 1000:.2f} ms)")
        print("=" * 60)

        if self.quest:
            print(f"VR 设备已连接: {self.quest}")
        else:
            print("未连接 VR 设备")

        print("\n操作说明:")
        print(f"  X 按钮: 按住开始{self.teleop_arm_mode_label}遥操作，松开停止")

        try:
            while not self.shutdown_event.is_set():
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n用户中断程序")
        finally:
            self.delete()

    def delete(self):
        print("清理资源...")
        self.shutdown_event.set()
        time.sleep(0.1)
        try:
            if hasattr(self, "arm_client"):
                self.arm_client.disconnect()
        except Exception as e:
            print(f"断开 ArmClient 时发生错误: {e}")
        print("资源清理完成")