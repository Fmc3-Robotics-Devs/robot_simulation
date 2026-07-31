#!/usr/bin/env python3
"""Validate the structure and a decoded sample of a local LeRobot v3 dataset."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from lerobot.datasets.lerobot_dataset import LeRobotDataset


def validate(
    root: Path,
    repo_id: str,
    expected_cameras: int,
    require_actions: bool,
    decode_sample: bool,
    expected_episodes: int | None = 1,
) -> dict:
    root = root.resolve()
    required = (
        root / "meta" / "info.json",
        root / "meta" / "stats.json",
        root / "meta" / "tasks.parquet",
    )
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"Missing LeRobot metadata: {missing}")

    dataset = LeRobotDataset(repo_id=repo_id, root=root, video_backend="pyav")
    version = dataset.meta.info["codebase_version"]
    if version != "v3.0":
        raise ValueError(f"Expected LeRobot v3.0, got {version}")
    if expected_episodes is not None and dataset.num_episodes != expected_episodes:
        raise ValueError(
            f"Expected {expected_episodes} episode(s), got {dataset.num_episodes}"
        )
    if dataset.num_frames < 1:
        raise ValueError("Dataset has no frames")
    if len(dataset.meta.video_keys) != expected_cameras:
        raise ValueError(
            f"Expected {expected_cameras} cameras, got {dataset.meta.video_keys}"
        )

    shapes = {
        key: tuple(dataset.features[key]["shape"])
        for key in dataset.meta.video_keys
    }
    if len(set(shapes.values())) != 1:
        raise ValueError(f"GR00T requires equal camera shapes, got {shapes}")

    has_state = "observation.state" in dataset.features
    has_action = "action" in dataset.features
    if require_actions and not (has_state and has_action):
        raise ValueError("Training validation requires observation.state and action")

    provenance_path = root / "meta" / "provenance.json"
    provenance = (
        json.loads(provenance_path.read_text()) if provenance_path.is_file() else {}
    )

    subtask_summary = None
    if "subtask_index" in dataset.features:
        import pandas as pd

        subtasks_path = root / "meta" / "subtasks.parquet"
        segments_path = root / "meta" / "subtask_segments.csv"
        for path in (subtasks_path, segments_path):
            if not path.is_file():
                raise FileNotFoundError(f"subtask_index feature without {path}")
        vocabulary = pd.read_parquet(subtasks_path)
        segments = pd.read_csv(segments_path)
        if segments["subtask_index"].max() >= len(vocabulary):
            raise ValueError("subtask_segments.csv references an unknown index")
        per_episode = segments.groupby("episode_index")["num_frames"].sum()
        episodes_meta = dataset.meta.episodes
        for episode_index, total in per_episode.items():
            try:
                expected = int(episodes_meta[int(episode_index)]["length"])
            except (KeyError, TypeError):
                expected = int(episodes_meta["length"][int(episode_index)])
            if int(total) != expected:
                raise ValueError(
                    f"episode {episode_index}: subtask segments cover "
                    f"{total} frames, episode has {expected}"
                )
        subtask_summary = {
            "vocabulary_size": int(len(vocabulary)),
            "segments": int(len(segments)),
            "segments_cover_all_frames": True,
        }

    task = dataset.meta.tasks.index[0]
    if decode_sample:
        sample = dataset[0]
        task = sample["task"]
        for key in dataset.meta.video_keys:
            if tuple(sample[key].shape) != (3, shapes[key][0], shapes[key][1]):
                raise ValueError(
                    f"Decoded {key} shape {tuple(sample[key].shape)} does not "
                    f"match metadata {shapes[key]}"
                )
        if "subtask_index" in dataset.features and "subtask_index" not in sample:
            raise ValueError("subtask_index feature missing from decoded sample")

    summary = {
        "root": str(root),
        "repo_id": repo_id,
        "codebase_version": version,
        "episodes": dataset.num_episodes,
        "frames": dataset.num_frames,
        "fps": dataset.fps,
        "video_keys": dataset.meta.video_keys,
        "video_shapes_hwc": shapes,
        "has_state": has_state,
        "has_action": has_action,
        "task": task,
        "training_ready": provenance.get(
            "training_ready", bool(has_state and has_action)
        ),
        "physics_valid": provenance.get("physics", {}).get("valid"),
        "subtasks": subtask_summary,
        "decoded_sample": decode_sample,
    }
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-id", default="local/franzi_machine_tending_v3")
    parser.add_argument("--expected-cameras", type=int, default=4)
    parser.add_argument(
        "--expected-episodes",
        type=int,
        default=None,
        help="精确的 episode 数;默认不限(旧的单 episode 语义传 1)。",
    )
    parser.add_argument("--require-actions", action="store_true")
    parser.add_argument("--decode-sample", action="store_true")
    args = parser.parse_args()
    print(
        json.dumps(
            validate(
                args.root,
                args.repo_id,
                args.expected_cameras,
                args.require_actions,
                args.decode_sample,
                args.expected_episodes,
            ),
            indent=2,
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
