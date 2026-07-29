"""Taught docking poses.

Under SLAM there is no prior map to look a station up in, so the docking poses
are taught once: a human drives the robot to each station and the pose is
recorded. That file is then the only thing the running task knows about where
stations are - and it only has to be roughly right, because the tag supplies
the precision (see the `dock_tolerance` tool for how rough is rough enough).

Poses live in the navigation frame. Teaching them again is the correct response
to the map changing, not editing them by hand.
"""

from pathlib import Path

import yaml

STATIONS_KEY = "docks"


class MissingDock(RuntimeError):
    pass


class DockBook:
    def __init__(self, poses, source):
        self._poses = poses
        self._source = source

    @classmethod
    def load(cls, path):
        path = Path(path)
        if not path.is_file():
            raise MissingDock(
                f"no taught docking poses at {path}; run `ros2 run franzi_pick_place "
                "teach_docks` first"
            )
        document = yaml.safe_load(path.read_text()) or {}
        entries = document.get(STATIONS_KEY) or {}
        poses = {
            station: (float(entry["x"]), float(entry["y"]), float(entry["yaw"]))
            for station, entry in entries.items()
        }
        return cls(poses, path)

    def pose(self, station):
        if station not in self._poses:
            raise MissingDock(
                f"station '{station}' was never taught (in {self._source}); "
                "drive there and record it"
            )
        return self._poses[station]

    @property
    def stations(self):
        return sorted(self._poses)


def save(path, poses, frame_id):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    document = {
        STATIONS_KEY: {
            station: {"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 5)}
            for station, (x, y, yaw) in sorted(poses.items())
        }
    }
    header = (
        "# Docking poses taught by driving the robot to each station.\n"
        f"# Frame: {frame_id}. Re-teach after the map changes; do not hand-edit.\n"
        "# Only coarse accuracy is needed - the station tag supplies the rest.\n"
    )
    path.write_text(header + yaml.safe_dump(document, sort_keys=False))
    return path
