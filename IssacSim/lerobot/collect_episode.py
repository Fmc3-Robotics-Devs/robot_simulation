#!/usr/bin/env python3
"""在 tending 任务运行时同步采集一条 LeRobot episode 的原始数据(ROS 侧)。

按 15 Hz 墙钟采样,与 Isaac 相机的 useSystemTime 时间戳同一条时间线:

* 四路相机(observer→overhead 缩放、head_d435 缩放、双腕中心裁剪)存 JPEG;
* `/joint_states` + `base/pose` 组成 54 维 state(速度为后处理有限差分);
* action(25 维)= 下一采样周期的关节位置 + 底盘体坐标 twist + 夹爪开度,
  在保存时由 state 序列后移一拍导出(kinematic 镜像阶段命令即测量);
* `task/state` 逐帧记录状态机状态,配合 subtask_labels.py 切 subtask 段;
* 物理随机化参数用与 Isaac 侧同一确定性采样(base_seed + episode_index)。

从状态离开 IDLE 开始录,DONE 后收尾一秒停止;FAULT 标记失败。用法:

    source /opt/ros/jazzy/setup.bash && source Rviz/install/setup.bash
    python3 IssacSim/lerobot/collect_episode.py \
        --output IssacSim/datasets/episodes --episode-index 0
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from sample_physics_params import sample_parameters  # noqa: E402

FPS = 15
FRAME_W, FRAME_H = 640, 360
SCHEMA = json.loads((HERE / "franzi_groot_schema.json").read_text())
STATE_NAMES = SCHEMA["state"]["names"]
ACTION_NAMES = SCHEMA["action"]["names"]
JOINT_ORDER = [n.split(".", 1)[1] for n in STATE_NAMES
               if n.startswith("joint_position.")]
TARGET_JOINTS = [n.split(".", 1)[1] for n in ACTION_NAMES
                 if n.startswith("joint_position_target.")]
GRIPPER_MAX_GAP = 0.095

CAMERA_TOPICS = {
    "overhead": "observer/color/image_raw",
    "head_d435": "head_d435/color/image_raw",
    "left_wrist_d405": "left_wrist_d405/color/image_raw",
    "right_wrist_d405": "right_wrist_d405/color/image_raw",
}

RECORD_END_STATES = {"DONE", "FAULT"}


def to_frame(image_bgr: np.ndarray) -> np.ndarray:
    """任意输入统一成 640x360:16:9 直接缩放,4:3 先中心裁剪。"""
    height, width = image_bgr.shape[:2]
    target_h = int(round(width * FRAME_H / FRAME_W))
    if height != target_h:
        top = max(0, (height - target_h) // 2)
        image_bgr = image_bgr[top:top + target_h]
    return cv2.resize(image_bgr, (FRAME_W, FRAME_H),
                      interpolation=cv2.INTER_AREA)


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True,
                        help="episodes 根目录")
    parser.add_argument("--episode-index", type=int, required=True)
    parser.add_argument("--fps", type=int, default=15,
                        help="采样率;须不高于 Isaac 实际渲染发布率")
    parser.add_argument("--max-minutes", type=float, default=12.0)
    parser.add_argument("--start-timeout", type=float, default=240.0,
                        help="等待任务离开 IDLE 的秒数")
    parser.add_argument("--jpeg-quality", type=int, default=92)
    parser.add_argument("--stale-warn-s", type=float, default=0.4)
    args = parser.parse_args()
    global FPS
    FPS = args.fps

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, JointState
    from geometry_msgs.msg import PoseStamped
    from franzi_engraving_interfaces.msg import TaskState

    episode_dir = args.output / f"episode_{args.episode_index:04d}"
    if episode_dir.exists():
        raise SystemExit(f"{episode_dir} 已存在,拒绝覆盖")
    for cam in CAMERA_TOPICS:
        (episode_dir / "frames" / cam).mkdir(parents=True)

    physics = sample_parameters(
        json.loads((HERE / "physics_collection.json").read_text()),
        args.episode_index,
    )

    class Collector(Node):
        def __init__(self):
            super().__init__("lerobot_collector")
            self.images: dict[str, tuple[float, np.ndarray]] = {}
            self.joints: dict[str, float] = {}
            self.joints_stamp = 0.0
            self.base = None            # (x, y, yaw)
            self.base_stamp = 0.0
            self.task_state = "IDLE"
            self.task_stamp = None      # 最近一次收到 task/state 的时刻
            self.rows = []              # 每帧: (t, joints快照, base, state)
            self.saved = 0
            self.stale = 0
            self.started = False
            self.done_state = None
            self.done_at = None
            self.t0 = time.time()
            sensor_qos = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            for cam, topic in CAMERA_TOPICS.items():
                self.create_subscription(
                    Image, topic,
                    lambda m, c=cam: self._on_image(c, m), sensor_qos)
            self.create_subscription(JointState, "joint_states",
                                     self._on_joints, 50)
            self.create_subscription(PoseStamped, "base/pose",
                                     self._on_base, 50)
            self.create_subscription(TaskState, "task/state",
                                     self._on_task, 10)
            self.create_timer(1.0 / FPS, self._tick)

        def _on_image(self, cam, message):
            data = np.frombuffer(message.data, dtype=np.uint8)
            image = data.reshape(message.height, message.width, -1)
            if message.encoding in ("rgb8", "rgba8"):
                image = cv2.cvtColor(
                    image[:, :, :3], cv2.COLOR_RGB2BGR)
            elif message.encoding in ("bgr8", "bgra8"):
                image = image[:, :, :3].copy()
            else:
                raise SystemExit(f"未处理的图像编码 {message.encoding}")
            self.images[cam] = (time.time(), image)

        def _on_joints(self, message):
            for name, position in zip(message.name, message.position):
                self.joints[name] = position
            self.joints_stamp = time.time()

        def _on_base(self, message):
            p, q = message.pose.position, message.pose.orientation
            self.base = (p.x, p.y, quat_to_yaw(q.x, q.y, q.z, q.w))
            self.base_stamp = time.time()

        def _on_task(self, message):
            if message.state != self.task_state:
                self.get_logger().info(
                    f"task {self.task_state} -> {message.state}")
            self.task_state = message.state
            self.task_stamp = time.time()

        def _tick(self):
            now = time.time()
            if not self.started:
                if self.task_state not in ("IDLE",):
                    self.started = True
                    self.get_logger().info("任务已启动,开始录制")
                elif now - self.t0 > args.start_timeout:
                    self.done_state = "START_TIMEOUT"
                    raise SystemExit(1)
                else:
                    return
            if self.done_at is not None:
                if now - self.done_at > 1.0:
                    rclpy.shutdown()
                return
            missing = [c for c in CAMERA_TOPICS if c not in self.images]
            if missing or not self.joints or self.base is None:
                if now - self.t0 > args.start_timeout:
                    raise SystemExit(f"数据源不齐: 缺 {missing or 'joints/base'}")
                return
            missing_joints = [j for j in JOINT_ORDER if j not in self.joints]
            if missing_joints:
                raise SystemExit(f"/joint_states 缺关节 {missing_joints}")
            ages = [now - self.images[c][0] for c in CAMERA_TOPICS]
            if max(ages) > args.stale_warn_s:
                self.stale += 1
            index = self.saved
            for cam in CAMERA_TOPICS:
                frame = to_frame(self.images[cam][1])
                cv2.imwrite(
                    str(episode_dir / "frames" / cam / f"{index:06d}.jpg"),
                    frame,
                    [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality],
                )
            self.rows.append((
                now,
                np.array([self.joints[j] for j in JOINT_ORDER],
                         dtype=np.float64),
                np.array(self.base, dtype=np.float64),
                self.task_state,
            ))
            self.saved += 1
            if self.saved % 150 == 0:
                self.get_logger().info(
                    f"{self.saved} 帧 | 状态 {self.task_state} | "
                    f"滞后帧 {self.stale}")
            if self.task_state in RECORD_END_STATES and self.done_at is None:
                self.done_state = self.task_state
                self.done_at = now
            # 任务管理器在 DONE 后立即退出,0.2s 的状态定时器常来不及发
            # 最后一拍:task/state 静默超过 5s 也视为周期结束——末状态在
            # 收尾段(GO_HOME_END/DONE)则判成功,否则是栈崩溃,判失败。
            if (self.done_at is None and self.task_stamp is not None
                    and now - self.task_stamp > 5.0):
                if self.task_state in ("GO_HOME_END", "RETURN_HOME", "DONE"):
                    self.done_state = "DONE_INFERRED"
                else:
                    self.done_state = f"STACK_SILENT_{self.task_state}"
                self.done_at = now
            if now - self.t0 > args.max_minutes * 60.0:
                self.done_state = self.done_state or "TIMEOUT"
                rclpy.shutdown()

    from rclpy.executors import ExternalShutdownException

    rclpy.init()
    collector = Collector()
    try:
        rclpy.spin(collector)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass

    rows = collector.rows
    if len(rows) < FPS * 5:
        raise SystemExit(f"只采到 {len(rows)} 帧,不构成 episode")

    times = np.array([r[0] for r in rows])
    times -= times[0]
    joints = np.stack([r[1] for r in rows])              # (N, 24)
    base = np.stack([r[2] for r in rows])                # (N, 3) x,y,yaw
    states_txt = [r[3] for r in rows]

    # ---- state 54 维 ----
    dt = np.gradient(times)
    dt[dt <= 0] = 1.0 / FPS
    base_vel_world = np.gradient(base, axis=0) / dt[:, None]
    # yaw 差分处理回绕
    yaw = base[:, 2]
    dyaw = np.diff(yaw)
    dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi
    wz = np.concatenate([[dyaw[0] / dt[0]], dyaw / dt[1:]])
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    vx = cos_y * base_vel_world[:, 0] + sin_y * base_vel_world[:, 1]
    vy = -sin_y * base_vel_world[:, 0] + cos_y * base_vel_world[:, 1]
    joint_vel = np.gradient(joints, axis=0) / dt[:, None]
    state = np.concatenate(
        [base, np.stack([vx, vy, wz], axis=1), joints, joint_vel], axis=1
    ).astype(np.float32)
    assert state.shape[1] == len(STATE_NAMES), state.shape

    # ---- action 25 维:下一拍的命令(末帧重复) ----
    nxt = np.arange(1, len(rows) + 1).clip(max=len(rows) - 1)
    joint_by_name = {name: joints[:, i] for i, name in enumerate(JOINT_ORDER)}
    target = np.stack([joint_by_name[j][nxt] for j in TARGET_JOINTS], axis=1)
    grip = {}
    for side in ("left", "right"):
        f1 = joint_by_name[f"{side}finger1_joint"]
        f2 = joint_by_name[f"{side}finger2_joint"]
        grip[side] = (GRIPPER_MAX_GAP - (f1 - f2))[nxt]
    action = np.concatenate(
        [np.stack([vx[nxt], vy[nxt], wz[nxt]], axis=1), target,
         np.stack([grip["left"], grip["right"]], axis=1)], axis=1
    ).astype(np.float32)
    assert action.shape[1] == len(ACTION_NAMES), action.shape

    np.savez_compressed(
        episode_dir / "telemetry.npz",
        state=state, action=action, timestamps_s=times,
        state_names=np.array(STATE_NAMES), action_names=np.array(ACTION_NAMES),
    )
    meta = {
        "episode_index": args.episode_index,
        "fps": FPS,
        "frame_width": FRAME_W,
        "frame_height": FRAME_H,
        "frames": len(rows),
        "duration_s": float(times[-1]),
        "success": collector.done_state in ("DONE", "DONE_INFERRED"),
        "final_state": collector.done_state,
        "stale_frames": collector.stale,
        "cameras": {cam: topic for cam, topic in CAMERA_TOPICS.items()},
        "frame_states": states_txt,
        "physics": physics,
        "physics_applied": ["workpiece_mass_kg", "static_friction",
                            "dynamic_friction", "restitution"],
        "physics_recorded_only": ["joint_stiffness_scale",
                                  "joint_damping_scale",
                                  "actuator_effort_scale",
                                  "gravity_magnitude_m_s2"],
        "collected_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    (episode_dir / "meta.json").write_text(
        json.dumps(meta, indent=2, ensure_ascii=False) + "\n")
    print(f"episode_{args.episode_index:04d}: {len(rows)} 帧 "
          f"{times[-1]:.1f}s success={meta['success']} "
          f"final={collector.done_state} stale={collector.stale}")
    if not meta["success"]:
        sys.exit(2)


if __name__ == "__main__":
    main()
