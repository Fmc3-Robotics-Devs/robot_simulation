#!/usr/bin/env python3
"""把 collect_episode.py 的原始 episode 目录打包成多 episode LeRobot v3.0 数据集。

相对旧的 convert_run_to_lerobot_v3.py(单 episode、切旧录像):本脚本消费
带 telemetry 的新采集,产出:

* 四路 640x360 视频特征 + 54 维 state + 25 维 action;
* 逐帧 ``subtask_index``(int64)列,由状态机状态经 subtask_labels.py 映射;
* ``meta/subtasks.parquet`` / ``meta/subtask_segments.csv``(与本机
  LW-BenchHub subtask_fine 数据集同一约定,boundary_source=state_machine);
* ``meta/physics_randomization.json``:每条 episode 的八项采样实值与
  applied / recorded-only 标记;
* ``meta/provenance.json``。

默认只收 success 的 episode;``--include-failed`` 才收失败条。用法:

    conda run -n lerobot-groot python \
      IssacSim/lerobot/build_lerobot_v3_dataset.py \
      --episodes IssacSim/datasets/episodes \
      --output IssacSim/datasets/franzi_machine_tending_physx_v3 \
      --repo-id local/franzi_machine_tending_physx_v3
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
from lerobot.datasets.lerobot_dataset import LeRobotDataset

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from subtask_labels import STATE_TO_SUBTASK, segments_from_states  # noqa: E402

FRAME_SHAPE = (360, 640, 3)
CAMERAS = ("overhead", "head_d435", "left_wrist_d405", "right_wrist_d405")
SCHEMA = json.loads((HERE / "franzi_groot_schema.json").read_text())
TASK = json.loads((HERE / "physics_collection.json").read_text())["dataset"]["task"]


def load_episode(path: Path) -> dict:
    meta = json.loads((path / "meta.json").read_text())
    with np.load(path / "telemetry.npz", allow_pickle=False) as npz:
        state = npz["state"].astype(np.float32)
        action = npz["action"].astype(np.float32)
        state_names = [str(v) for v in npz["state_names"].tolist()]
        action_names = [str(v) for v in npz["action_names"].tolist()]
    if state_names != SCHEMA["state"]["names"]:
        raise ValueError(f"{path}: state 名与 schema 不一致")
    if action_names != SCHEMA["action"]["names"]:
        raise ValueError(f"{path}: action 名与 schema 不一致")
    frames = meta["frames"]
    if not (len(state) == len(action) == frames == len(meta["frame_states"])):
        raise ValueError(f"{path}: 帧数不一致")
    for cam in CAMERAS:
        count = len(list((path / "frames" / cam).glob("*.jpg")))
        if count != frames:
            raise ValueError(f"{path}: {cam} 有 {count} 张图,期望 {frames}")
    return dict(path=path, meta=meta, state=state, action=action)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repo-id", default="local/franzi_machine_tending_physx_v3")
    parser.add_argument("--include-failed", action="store_true")
    parser.add_argument("--vcodec", default="h264")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper()),
                        format="%(asctime)s %(levelname)s %(message)s")

    if args.output.exists():
        raise SystemExit(f"{args.output} 已存在,拒绝覆盖")
    episode_dirs = sorted(args.episodes.glob("episode_*"))
    episodes = []
    for path in episode_dirs:
        if not (path / "telemetry.npz").exists():
            logging.warning("跳过 %s(无 telemetry)", path.name)
            continue
        episode = load_episode(path)
        if not episode["meta"]["success"] and not args.include_failed:
            logging.warning("跳过 %s(success=false)", path.name)
            continue
        episodes.append(episode)
    if not episodes:
        raise SystemExit("没有可用的 episode")
    fps_values = {int(e["meta"]["fps"]) for e in episodes}
    if len(fps_values) != 1:
        raise SystemExit(f"episodes 的 fps 不一致: {sorted(fps_values)}")
    fps = fps_values.pop()

    # subtask 词表:全表固定顺序,与出现与否无关,保证跨数据集稳定。
    vocab = sorted(set(STATE_TO_SUBTASK.values()))
    subtask_to_index = {s: i for i, s in enumerate(vocab)}

    features = {
        f"observation.images.{cam}": {
            "dtype": "video",
            "shape": FRAME_SHAPE,
            "names": ["height", "width", "channels"],
        }
        for cam in CAMERAS
    }
    features["observation.state"] = {
        "dtype": "float32",
        "shape": (len(SCHEMA["state"]["names"]),),
        "names": SCHEMA["state"]["names"],
    }
    features["action"] = {
        "dtype": "float32",
        "shape": (len(SCHEMA["action"]["names"]),),
        "names": SCHEMA["action"]["names"],
    }
    features["subtask_index"] = {
        "dtype": "int64", "shape": (1,), "names": None,
    }

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        root=args.output.resolve(),
        fps=fps,
        robot_type="franzi_mobile_manipulator",
        features=features,
        use_videos=True,
        vcodec=args.vcodec,
    )

    segment_rows = []
    physics_meta = []
    for new_index, episode in enumerate(episodes):
        meta = episode["meta"]
        frame_states = meta["frame_states"]
        segments = segments_from_states(frame_states)
        per_frame_subtask = np.empty(len(frame_states), dtype=np.int64)
        for segment in segments:
            per_frame_subtask[segment["start"]:segment["end_inclusive"] + 1] \
                = subtask_to_index[segment["subtask"]]
        for index in range(meta["frames"]):
            frame = {}
            for cam in CAMERAS:
                image_bgr = cv2.imread(
                    str(episode["path"] / "frames" / cam / f"{index:06d}.jpg"))
                if image_bgr is None or image_bgr.shape[:2] != FRAME_SHAPE[:2]:
                    raise RuntimeError(
                        f"{episode['path'].name}/{cam}/{index:06d}.jpg 坏帧")
                frame[f"observation.images.{cam}"] = cv2.cvtColor(
                    image_bgr, cv2.COLOR_BGR2RGB)
            frame["observation.state"] = episode["state"][index]
            frame["action"] = episode["action"][index]
            frame["subtask_index"] = per_frame_subtask[index:index + 1]
            frame["task"] = TASK
            dataset.add_frame(frame)
        dataset.save_episode()
        logging.info("episode %d (%s): %d 帧, %d 个 subtask 段",
                     new_index, episode["path"].name, meta["frames"],
                     len(segments))
        for segment in segments:
            start, end = segment["start"], segment["end_inclusive"]
            segment_rows.append({
                "episode_index": new_index,
                "source_episode_index": meta["episode_index"],
                "task_index": 0,
                "task": TASK,
                "subtask": segment["subtask"],
                "subtask_index": subtask_to_index[segment["subtask"]],
                "start_frame_index": start,
                "end_frame_index_inclusive": end,
                "num_frames": end - start + 1,
                "start_timestamp": start / fps,
                "end_timestamp": end / fps,
                "boundary_source": "state_machine",
            })
        physics_meta.append({
            "episode_index": new_index,
            "source_episode_index": meta["episode_index"],
            "success": meta["success"],
            "frames": meta["frames"],
            "duration_s": meta["duration_s"],
            "stale_frames": meta.get("stale_frames"),
            "physics": meta["physics"],
            "physics_applied": meta["physics_applied"],
            "physics_recorded_only": meta["physics_recorded_only"],
        })
    dataset.finalize()

    meta_dir = args.output.resolve() / "meta"
    pd.DataFrame(
        {"subtask": vocab, "subtask_index": range(len(vocab))}
    ).set_index("subtask").to_parquet(meta_dir / "subtasks.parquet")
    pd.DataFrame(segment_rows).to_csv(
        meta_dir / "subtask_segments.csv", index=False)
    (meta_dir / "physics_randomization.json").write_text(json.dumps({
        "config": json.loads((HERE / "physics_collection.json").read_text()),
        "note": ("Kinematic mirror stage: workpiece mass/friction/restitution "
                 "are authored on the stage; joint stiffness/damping/effort "
                 "and gravity are recorded for the future dynamics stage but "
                 "do not affect the mirrored motion."),
        "episodes": physics_meta,
    }, indent=2, ensure_ascii=False) + "\n")
    (meta_dir / "provenance.json").write_text(json.dumps({
        "dataset_format": "LeRobot v3.0",
        "training_ready": True,
        "task": TASK,
        "fps": fps,
        "episodes": len(episodes),
        "subtask_labeling": {
            "boundary_source": "franzi_task_manager task/state transitions",
            "mapping_module": "IssacSim/lerobot/subtask_labels.py",
            "per_frame_feature": "subtask_index",
        },
        "action_semantics": SCHEMA["action"]["semantics"],
        "state_semantics": SCHEMA["state"]["semantics"],
        "physics": {
            "valid": False,
            "engine": None,
            "reason": "Kinematic mirror stage: per-episode sampled parameters "
                      "are authored on the stage / recorded in "
                      "physics_randomization.json but do not drive the "
                      "mirrored motion yet.",
        },
        "source_episode_dirs": [str(e["path"]) for e in episodes],
        "collector": "IssacSim/lerobot/collect_episode.py",
        "backend": "Isaac Sim RTX rendering, kinematic ROS mirror, "
                   "real MoveIt/Nav control stack",
    }, indent=2, ensure_ascii=False) + "\n")
    logging.info("写入 %s: %d episodes, %d subtask 段, 词表 %d",
                 args.output, len(episodes), len(segment_rows), len(vocab))


if __name__ == "__main__":
    main()
