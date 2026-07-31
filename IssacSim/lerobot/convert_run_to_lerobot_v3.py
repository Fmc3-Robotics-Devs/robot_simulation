#!/usr/bin/env python3
"""Convert one synchronized Isaac render run to a LeRobot v3.0 episode.

The historical ``run_*`` folders contain a fixed CCTV video and a 2x2
on-board-camera mosaic.  They do not contain robot state or action telemetry.
For that reason this converter refuses to make a training episode unless a
telemetry NPZ is supplied, or ``--visual-only`` is explicitly requested for a
format/vision smoke test.

Expected telemetry NPZ keys:

``state``
    Float array with shape ``(N, state_dim)``.
``action``
    Float array with shape ``(N, action_dim)``.
``timestamps_s`` (optional)
    Monotonic float timestamps for resampling telemetry to video time.
``state_names`` and ``action_names`` (optional)
    Unicode arrays naming each vector component.
"""

from __future__ import annotations

import argparse
import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Final

import cv2
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset


DEFAULT_TASK: Final = (
    "Complete one engraving-machine tending cycle: navigate from home to the "
    "feeder, pick up a raw aluminum workpiece, load it into the machine "
    "fixture, wait for machining, unload the finished part, place it in the "
    "next empty outfeed slot, and return home."
)
DEFAULT_CAMERAS: Final = (
    "overhead",
    "head_d435",
    "left_wrist_d405",
    "right_wrist_d405",
)
FRAME_HEIGHT: Final = 360
FRAME_WIDTH: Final = 640
FRAME_SHAPE: Final = (FRAME_HEIGHT, FRAME_WIDTH, 3)
BASE_SEED: Final = 12345678


@dataclass(frozen=True)
class VideoInfo:
    path: Path
    fps: float
    frames: int
    width: int
    height: int


@dataclass(frozen=True)
class Telemetry:
    state: np.ndarray
    action: np.ndarray
    state_names: list[str]
    action_names: list[str]


def _video_info(path: Path) -> VideoInfo:
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise ValueError(f"Cannot open video: {path}")
    try:
        return VideoInfo(
            path=path,
            fps=float(capture.get(cv2.CAP_PROP_FPS)),
            frames=int(capture.get(cv2.CAP_PROP_FRAME_COUNT)),
            width=int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            height=int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        )
    finally:
        capture.release()


def _parse_cameras(value: str) -> tuple[str, ...]:
    cameras = tuple(part.strip() for part in value.split(",") if part.strip())
    supported = {
        "overhead",
        "head_d435",
        "body_d435",
        "left_wrist_d405",
        "right_wrist_d405",
    }
    unknown = set(cameras) - supported
    if unknown:
        raise argparse.ArgumentTypeError(
            f"Unknown camera(s): {sorted(unknown)}; choose from {sorted(supported)}"
        )
    if len(cameras) != 4 or len(set(cameras)) != 4:
        raise argparse.ArgumentTypeError("Exactly four distinct cameras are required")
    return cameras


def _extract_views(
    cctv_bgr: np.ndarray,
    mosaic_bgr: np.ndarray,
    cameras: tuple[str, ...],
) -> dict[str, np.ndarray]:
    if cctv_bgr.shape[:2] != (720, 1280):
        raise ValueError(f"Expected CCTV 1280x720, got {cctv_bgr.shape[1]}x{cctv_bgr.shape[0]}")
    if mosaic_bgr.shape[:2] != (720, 1280):
        raise ValueError(
            f"Expected on-board mosaic 1280x720, got "
            f"{mosaic_bgr.shape[1]}x{mosaic_bgr.shape[0]}"
        )

    # The compositor command was not retained with the old recording.  This
    # ordering is inferred from the documented four physical cameras and
    # visual inspection of the tiles.
    bgr_views = {
        "overhead": cv2.resize(
            cctv_bgr,
            (FRAME_WIDTH, FRAME_HEIGHT),
            interpolation=cv2.INTER_AREA,
        ),
        "head_d435": mosaic_bgr[0:360, 0:640],
        "body_d435": mosaic_bgr[0:360, 640:1280],
        "left_wrist_d405": mosaic_bgr[360:720, 0:640],
        "right_wrist_d405": mosaic_bgr[360:720, 640:1280],
    }
    return {
        f"observation.images.{name}": cv2.cvtColor(bgr_views[name], cv2.COLOR_BGR2RGB)
        for name in cameras
    }


def _names(npz: np.lib.npyio.NpzFile, key: str, dimension: int) -> list[str]:
    if key not in npz:
        prefix = "state" if key == "state_names" else "action"
        return [f"{prefix}_{index}" for index in range(dimension)]
    names = [str(value) for value in npz[key].tolist()]
    if len(names) != dimension:
        raise ValueError(f"{key} has {len(names)} names for a {dimension}-D vector")
    return names


def _interpolate(values: np.ndarray, source_t: np.ndarray, target_t: np.ndarray) -> np.ndarray:
    columns = [
        np.interp(target_t, source_t, values[:, column])
        for column in range(values.shape[1])
    ]
    return np.stack(columns, axis=1).astype(np.float32)


def _load_telemetry(
    path: Path,
    frame_count: int,
    fps: int,
    offset_s: float,
) -> Telemetry:
    with np.load(path, allow_pickle=False) as npz:
        missing = {"state", "action"} - set(npz.files)
        if missing:
            raise ValueError(f"{path} is missing required arrays: {sorted(missing)}")

        state = np.asarray(npz["state"], dtype=np.float32)
        action = np.asarray(npz["action"], dtype=np.float32)
        if state.ndim != 2 or action.ndim != 2:
            raise ValueError("state and action must both be rank-2 arrays")
        if state.shape[0] != action.shape[0]:
            raise ValueError(
                f"state/action row mismatch: {state.shape[0]} != {action.shape[0]}"
            )
        if state.shape[1] > 64:
            raise ValueError(f"GR00T default max_state_dim is 64, got {state.shape[1]}")
        if action.shape[1] > 32:
            raise ValueError(f"GR00T default max_action_dim is 32, got {action.shape[1]}")

        state_names = _names(npz, "state_names", state.shape[1])
        action_names = _names(npz, "action_names", action.shape[1])
        target_t = np.arange(frame_count, dtype=np.float64) / fps + offset_s

        if "timestamps_s" in npz:
            source_t = np.asarray(npz["timestamps_s"], dtype=np.float64)
            if source_t.ndim != 1 or len(source_t) != state.shape[0]:
                raise ValueError("timestamps_s must have one entry per telemetry row")
            if np.any(np.diff(source_t) <= 0.0):
                raise ValueError("timestamps_s must be strictly increasing")
            if target_t[0] < source_t[0] or target_t[-1] > source_t[-1]:
                raise ValueError(
                    "Video time is outside telemetry coverage after applying "
                    f"offset {offset_s:.6f}s"
                )
            state = _interpolate(state, source_t, target_t)
            action = _interpolate(action, source_t, target_t)
        elif state.shape[0] != frame_count:
            raise ValueError(
                "Without timestamps_s, telemetry rows must equal video frames: "
                f"{state.shape[0]} != {frame_count}"
            )

    return Telemetry(state, action, state_names, action_names)


def _features(
    cameras: tuple[str, ...],
    telemetry: Telemetry | None,
) -> dict[str, dict]:
    features: dict[str, dict] = {
        f"observation.images.{name}": {
            "dtype": "video",
            "shape": FRAME_SHAPE,
            "names": ["height", "width", "channels"],
        }
        for name in cameras
    }
    if telemetry is not None:
        features["observation.state"] = {
            "dtype": "float32",
            "shape": (telemetry.state.shape[1],),
            "names": telemetry.state_names,
        }
        features["action"] = {
            "dtype": "float32",
            "shape": (telemetry.action.shape[1],),
            "names": telemetry.action_names,
        }
    return features


def _write_provenance(
    output: Path,
    *,
    run_dir: Path,
    cctv: VideoInfo,
    mosaic: VideoInfo,
    cameras: tuple[str, ...],
    frame_count: int,
    task: str,
    telemetry_path: Path | None,
    action_semantics: str | None,
) -> None:
    training_ready = telemetry_path is not None
    provenance = {
        "dataset_format": "LeRobot v3.0",
        "training_ready": training_ready,
        "visual_only": not training_ready,
        "task": task,
        "requested_base_seed": BASE_SEED,
        "source_seed": None,
        "source": {
            "run_directory": str(run_dir.resolve()),
            "cctv_video": str(cctv.path.resolve()),
            "onboard_mosaic_video": str(mosaic.path.resolve()),
            "source_fps": cctv.fps,
            "converted_frames": frame_count,
        },
        "camera_mapping": {
            "overhead": "cctv.mp4, full frame resized to 640x360",
            "head_d435": "onboard_quad.mp4, inferred top-left tile",
            "body_d435": "onboard_quad.mp4, inferred top-right tile",
            "left_wrist_d405": "onboard_quad.mp4, inferred bottom-left tile",
            "right_wrist_d405": "onboard_quad.mp4, inferred bottom-right tile",
            "selected": list(cameras),
        },
        "telemetry": {
            "path": str(telemetry_path.resolve()) if telemetry_path else None,
            "action_semantics": action_semantics,
        },
        "physics": {
            "valid": False,
            "engine": None,
            "reason": (
                "The historical source run was a kinematic ROS-to-Isaac mirror "
                "and did not record physical state/action telemetry."
            ),
        },
        "warnings": (
            []
            if training_ready
            else [
                "No action or robot-state labels are present.",
                "Do not use this visual-only episode to train the GR00T action head.",
                "The old 2x2 video does not retain raw camera calibration or timestamps.",
            ]
        ),
    }
    path = output / "meta" / "provenance.json"
    path.write_text(json.dumps(provenance, indent=2, ensure_ascii=False) + "\n")


def convert(args: argparse.Namespace) -> Path:
    run_dir = args.input_run.resolve()
    cctv_path = run_dir / "cctv.mp4"
    mosaic_path = run_dir / "onboard_quad.mp4"
    if not cctv_path.is_file() or not mosaic_path.is_file():
        raise FileNotFoundError(
            f"{run_dir} must contain cctv.mp4 and onboard_quad.mp4"
        )
    if args.output.exists():
        raise FileExistsError(
            f"Output already exists: {args.output}. Choose a fresh path; "
            "the converter never overwrites datasets."
        )
    if args.telemetry is None and not args.visual_only:
        raise ValueError(
            "This source run has no action/state labels. Supply --telemetry, "
            "or explicitly pass --visual-only for a non-training smoke test."
        )
    if args.telemetry is not None and args.visual_only:
        raise ValueError("--telemetry and --visual-only are mutually exclusive")
    if args.telemetry is not None and not args.action_semantics:
        raise ValueError("--action-semantics is required with --telemetry")

    cctv_info = _video_info(cctv_path)
    mosaic_info = _video_info(mosaic_path)
    if cctv_info.width != 1280 or cctv_info.height != 720:
        raise ValueError(f"Unexpected CCTV geometry: {cctv_info}")
    if mosaic_info.width != 1280 or mosaic_info.height != 720:
        raise ValueError(f"Unexpected on-board geometry: {mosaic_info}")
    if abs(cctv_info.fps - args.fps) > 1e-3:
        raise ValueError(f"CCTV fps {cctv_info.fps} != requested {args.fps}")
    if abs(mosaic_info.fps - args.fps) > 1e-3:
        raise ValueError(f"On-board fps {mosaic_info.fps} != requested {args.fps}")

    frame_count = min(cctv_info.frames, mosaic_info.frames)
    if args.max_frames is not None:
        frame_count = min(frame_count, args.max_frames)
    if frame_count < 1:
        raise ValueError("No frames selected")

    telemetry = (
        _load_telemetry(
            args.telemetry.resolve(),
            frame_count,
            args.fps,
            args.telemetry_offset_s,
        )
        if args.telemetry is not None
        else None
    )

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        root=args.output.resolve(),
        fps=args.fps,
        robot_type="franzi_mobile_manipulator",
        features=_features(args.cameras, telemetry),
        use_videos=True,
        vcodec=args.vcodec,
        streaming_encoding=True,
        encoder_queue_maxsize=args.encoder_queue_size,
        encoder_threads=args.encoder_threads,
    )
    cctv_capture = cv2.VideoCapture(str(cctv_path))
    mosaic_capture = cv2.VideoCapture(str(mosaic_path))

    saved = False
    converted = 0
    try:
        for index in range(frame_count):
            cctv_ok, cctv_frame = cctv_capture.read()
            mosaic_ok, mosaic_frame = mosaic_capture.read()
            if not cctv_ok or not mosaic_ok:
                raise RuntimeError(
                    f"Decode stopped at frame {index}: "
                    f"cctv_ok={cctv_ok}, mosaic_ok={mosaic_ok}"
                )

            frame = _extract_views(cctv_frame, mosaic_frame, args.cameras)
            frame["task"] = args.task
            if telemetry is not None:
                frame["observation.state"] = telemetry.state[index]
                frame["action"] = telemetry.action[index]
            dataset.add_frame(frame)
            converted += 1
            if converted == 1 or converted % args.progress_every == 0:
                logging.info("Converted %d/%d frames", converted, frame_count)

        dataset.save_episode(parallel_encoding=True)
        dataset.finalize()
        saved = True
    finally:
        cctv_capture.release()
        mosaic_capture.release()
        if not saved:
            try:
                dataset.finalize()
            except Exception:
                logging.exception("Could not finalize the incomplete dataset")

    _write_provenance(
        args.output.resolve(),
        run_dir=run_dir,
        cctv=cctv_info,
        mosaic=mosaic_info,
        cameras=args.cameras,
        frame_count=converted,
        task=args.task,
        telemetry_path=args.telemetry,
        action_semantics=args.action_semantics,
    )
    logging.info("Wrote LeRobot v3.0 episode to %s", args.output.resolve())
    return args.output.resolve()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repo-id", default="local/franzi_machine_tending_v3")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument(
        "--cameras",
        type=_parse_cameras,
        default=DEFAULT_CAMERAS,
        help=(
            "Comma-separated four-camera set. Default: "
            + ",".join(DEFAULT_CAMERAS)
        ),
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--telemetry", type=Path)
    mode.add_argument(
        "--visual-only",
        action="store_true",
        help="Create a format/vision smoke test without state or action labels.",
    )
    parser.add_argument(
        "--action-semantics",
        help="Required human-readable definition of the action vector.",
    )
    parser.add_argument("--telemetry-offset-s", type=float, default=0.0)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument(
        "--vcodec",
        choices=(
            "h264",
            "hevc",
            "libsvtav1",
            "auto",
            "h264_nvenc",
            "hevc_nvenc",
            "h264_vaapi",
            "h264_qsv",
        ),
        default="h264",
    )
    parser.add_argument("--encoder-threads", type=int, default=2)
    parser.add_argument("--encoder-queue-size", type=int, default=128)
    parser.add_argument("--progress-every", type=int, default=300)
    parser.add_argument("--log-level", default="INFO")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    if args.max_frames is not None and args.max_frames < 1:
        parser.error("--max-frames must be positive")
    if args.progress_every < 1:
        parser.error("--progress-every must be positive")
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s %(levelname)s %(message)s",
    )
    convert(args)


if __name__ == "__main__":
    main()
