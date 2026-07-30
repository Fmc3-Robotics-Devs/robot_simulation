#!/usr/bin/env python3
"""Compose synchronized Wheel Bot camera PNG sequences into a review video.

The script deliberately has no Isaac Sim dependency: it turns pre-captured PNG
frames into an easy-to-review 1920x1080 board and delegates H.264 encoding to
the system ``ffmpeg`` binary.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Mapping, Sequence

from PIL import Image, ImageDraw, ImageFont


CANVAS_SIZE = (1920, 1080)
VIEW_ORDER = ("overview", "head", "chest", "left", "right")
VIEW_LABELS = {
    "overview": "OVERVIEW",
    "head": "HEAD D435",
    "chest": "CHEST D435",
    "left": "LEFT WRIST D405",
    "right": "RIGHT WRIST D405",
}
FONT_REGULAR = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
FONT_BOLD = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf")


def _font(size: int, *, bold: bool = False) -> ImageFont.ImageFont:
    """Use a readable system font while retaining a portable fallback."""

    path = FONT_BOLD if bold else FONT_REGULAR
    try:
        return ImageFont.truetype(str(path), size)
    except OSError:
        return ImageFont.load_default()


def png_frames(directory: Path) -> list[Path]:
    """Return a deterministic frame list, rejecting a missing/empty directory."""
    if not directory.is_dir():
        raise ValueError(f"{directory}: PNG sequence directory does not exist")
    frames = sorted(path for path in directory.iterdir() if path.suffix.lower() == ".png")
    if not frames:
        raise ValueError(f"{directory}: no PNG frames found")
    return frames


def collect_sequences(directories: Mapping[str, Path]) -> dict[str, list[Path]]:
    """Load five camera sequences and prove they are frame-synchronous."""
    sequences = {name: png_frames(directories[name]) for name in VIEW_ORDER}
    lengths = {name: len(frames) for name, frames in sequences.items()}
    if len(set(lengths.values())) != 1:
        detail = ", ".join(f"{name}={count}" for name, count in lengths.items())
        raise ValueError(f"PNG sequence counts must match ({detail})")
    expected_names = [path.name for path in sequences["overview"]]
    for name, frames in sequences.items():
        names = [path.name for path in frames]
        if names != expected_names:
            raise ValueError(
                f"{name}: PNG filenames do not match the overview sequence"
            )
    return sequences


def _fit(image: Image.Image, size: tuple[int, int]) -> Image.Image:
    """Letterbox an image without changing its camera aspect ratio."""
    image = image.convert("RGB")
    scale = min(size[0] / image.width, size[1] / image.height)
    rendered = image.resize((round(image.width * scale), round(image.height * scale)), Image.Resampling.LANCZOS)
    tile = Image.new("RGB", size, "#10151d")
    tile.paste(rendered, ((size[0] - rendered.width) // 2, (size[1] - rendered.height) // 2))
    return tile


def _paste_labeled(canvas: Image.Image, image: Image.Image, box: tuple[int, int, int, int], label: str) -> None:
    """Paste one view with a compact label bar so exported frames stand alone."""
    x, y, width, height = box
    canvas.paste(_fit(image, (width, height)), (x, y))
    draw = ImageDraw.Draw(canvas)
    draw.rectangle((x, y, x + width, y + 34), fill="#0b1119")
    draw.text((x + 10, y + 6), label, fill="white", font=_font(18, bold=True))
    draw.rectangle((x, y, x + width - 1, y + height - 1), outline="#3b5268", width=2)


def compose_frame(
    images: Mapping[str, Image.Image],
    frame_number: int = 0,
    metadata: Mapping[str, object] | None = None,
) -> Image.Image:
    """Create the 1920x1080 "bottom-strip" review layout from five images.

    A large overview fills the upper-left working area.  Four 16:9 camera
    feeds form the lower strip, retaining every source pixel's aspect ratio.
    """
    missing = set(VIEW_ORDER) - set(images)
    if missing:
        raise ValueError(f"missing images for: {', '.join(sorted(missing))}")
    canvas = Image.new("RGB", CANVAS_SIZE, "#080c12")
    # The overview stays large enough to assess full-body motion and box pose.
    _paste_labeled(canvas, images["overview"], (0, 0, 1280, 720), VIEW_LABELS["overview"])
    draw = ImageDraw.Draw(canvas)
    draw.rectangle((1280, 0, 1920, 720), fill="#0b1119")
    metadata = metadata or {}
    stage = str(metadata.get("stage", "CAPTURE")).replace("_", " ").upper()
    elapsed_s = float(metadata.get("time_s", 0.0))
    progress = min(1.0, max(0.0, float(metadata.get("progress", 0.0))))
    attached = bool(metadata.get("box_attached", False))
    control_mode = str(
        metadata.get(
            "box_control_mode",
            "gripper" if attached else "dynamic",
        )
    )
    control_labels = {
        "gripper": ("GRIPPER", "#61d6a4"),
        "target_hold": ("TARGET HOLD", "#51a7e8"),
        "dynamic": ("DYNAMIC", "#f0bd68"),
    }
    control_label, control_color = control_labels.get(
        control_mode,
        (control_mode.upper(), "#f0bd68"),
    )
    draw.text((1320, 70), "WHEEL BOT", fill="white", font=_font(42, bold=True))
    draw.text((1320, 125), "BOX TRANSFER", fill="#9fc9eb", font=_font(28, bold=True))
    draw.text((1320, 208), "CURRENT STAGE", fill="#8295a8", font=_font(18, bold=True))
    draw.text((1320, 240), stage, fill="#ffffff", font=_font(30, bold=True))
    draw.text(
        (1320, 310),
        f"{elapsed_s:05.1f} s  ·  FRAME {frame_number:05d}",
        fill="#aeb9c5",
        font=_font(20),
    )
    draw.text(
        (1320, 365),
        "BOX CONTROL: " + control_label,
        fill=control_color,
        font=_font(20, bold=True),
    )
    box_position = metadata.get("box_position_m")
    if isinstance(box_position, (list, tuple)) and len(box_position) == 3:
        draw.text(
            (1320, 415),
            "BOX XYZ  "
            + "  ".join(f"{float(value):+.3f}" for value in box_position),
            fill="#c8d2dc",
            font=_font(18),
        )
    draw.rectangle((1320, 500, 1840, 522), fill="#263440")
    draw.rectangle((1320, 500, 1320 + round(520 * progress), 522), fill="#51a7e8")
    draw.text(
        (1320, 540),
        f"RUN PROGRESS  {progress * 100:05.1f}%",
        fill="#aeb9c5",
        font=_font(18, bold=True),
    )
    draw.text((1320, 635), "5 VIEWS · FRAME-SYNCHRONIZED", fill="#61d6a4", font=_font(17, bold=True))
    draw.rectangle((0, 720, 1920, 810), fill="#0b1119")
    draw.text(
        (28, 746),
        f"{stage}  ·  ISAAC SIM RTX  ·  CONTROLLED-ATTACHMENT SIL DEMO",
        fill="#d7e0e8",
        font=_font(22, bold=True),
    )
    for index, name in enumerate(("head", "chest", "left", "right")):
        _paste_labeled(canvas, images[name], (index * 480, 810, 480, 270), VIEW_LABELS[name])
    return canvas


def write_frames(
    sequences: Mapping[str, Sequence[Path]],
    frame_directory: Path,
    timeline_frames: Sequence[Mapping[str, object]] | None = None,
) -> int:
    """Render one composite PNG for each synchronized capture instant."""
    frame_directory.mkdir(parents=True, exist_ok=True)
    count = len(sequences["overview"])
    if timeline_frames is not None and len(timeline_frames) != count:
        raise ValueError(
            f"timeline contains {len(timeline_frames)} frames, expected {count}"
        )
    for index in range(count):
        with Image.open(sequences["overview"][index]) as overview, \
             Image.open(sequences["head"][index]) as head, \
             Image.open(sequences["chest"][index]) as chest, \
             Image.open(sequences["left"][index]) as left, \
             Image.open(sequences["right"][index]) as right:
            composite = compose_frame(
                {"overview": overview, "head": head, "chest": chest, "left": left, "right": right},
                index,
                timeline_frames[index] if timeline_frames is not None else None,
            )
            composite.save(frame_directory / f"frame_{index:05d}.png")
    return count


def load_timeline(path: Path, expected_count: int) -> list[dict[str, object]]:
    """Load and count-check the recorder's per-frame task metadata."""

    payload = json.loads(path.read_text(encoding="utf-8"))
    frames = payload.get("frames") if isinstance(payload, dict) else None
    if not isinstance(frames, list) or not all(isinstance(frame, dict) for frame in frames):
        raise ValueError(f"{path}: expected an object containing a frames list")
    if len(frames) != expected_count:
        raise ValueError(
            f"{path}: timeline contains {len(frames)} frames, expected {expected_count}"
        )
    return frames


def encode_mp4(frame_directory: Path, output: Path, fps: int) -> None:
    """Encode frames using ffmpeg, failing clearly if it is not installed."""
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise RuntimeError("ffmpeg is required to create MP4 output but was not found on PATH")
    output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        ffmpeg,
        "-y",
        "-framerate",
        str(fps),
        "-i",
        str(frame_directory / "frame_%05d.png"),
        "-c:v",
        "libx264",
        "-preset",
        "medium",
        "-crf",
        "18",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output),
    ]
    try:
        subprocess.run(command, check=True, capture_output=True, text=True)
    except subprocess.CalledProcessError as error:
        raise RuntimeError(f"ffmpeg encoding failed: {error.stderr[-1000:]}") from error


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in VIEW_ORDER:
        parser.add_argument(f"--{name}", type=Path, required=True, help=f"Directory of {name} PNG frames")
    parser.add_argument("--frames-dir", type=Path, help="Directory to retain composed PNG frames")
    parser.add_argument("--output", type=Path, help="MP4 output path")
    parser.add_argument("--timeline", type=Path, help="Optional recorder timeline JSON")
    parser.add_argument("--fps", type=int, default=15, help="Output frame rate (default: 15)")
    arguments = parser.parse_args(argv)
    if not arguments.frames_dir and not arguments.output:
        parser.error("provide --frames-dir and/or --output")
    if arguments.fps <= 0:
        parser.error("--fps must be positive")
    return arguments


def main(argv: Sequence[str] | None = None) -> int:
    """CLI entry point; return non-zero instead of creating partial silent output."""
    try:
        args = parse_args(argv)
        sequences = collect_sequences({name: getattr(args, name) for name in VIEW_ORDER})
        timeline = (
            load_timeline(args.timeline, len(sequences["overview"]))
            if args.timeline
            else None
        )
        temporary: tempfile.TemporaryDirectory[str] | None = None
        if args.frames_dir:
            rendered_frames = args.frames_dir
        else:
            temporary = tempfile.TemporaryDirectory(prefix="wheelbot_multiview_")
            rendered_frames = Path(temporary.name)
        count = write_frames(sequences, rendered_frames, timeline)
        if args.output:
            encode_mp4(rendered_frames, args.output, args.fps)
        if temporary:
            temporary.cleanup()
        print(f"composed {count} synchronized frames")
        return 0
    except (ValueError, RuntimeError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
