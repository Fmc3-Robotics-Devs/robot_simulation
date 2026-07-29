"""The machine-tending cell, read from the ROS side's own configuration.

`task.yaml` is the single source of truth for the layout. Isaac reads it rather
than restating it, so the two simulators cannot drift apart: a pose computed on
the ROS side is directly usable here because the Isaac world frame is the ROS
`odom` frame, same origin, same axes, same numbers.
"""

from dataclasses import dataclass
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
TASK_CONFIG = REPO / "Rviz" / "src" / "franzi_pick_place" / "config" / "task.yaml"

STATIONS = ("feeder", "machine", "outfeed")

# Body posture `work` and head posture `look_down`, from the SRDF.
WORK_POSTURE = {
    "calf_pitch_joint": -0.488692191,
    "thigh_pitch_joint": 0.977384381,
    "waist_pitch_joint": -0.488692191,
    "waist_yaw_joint": 0.0,
}
LOOK_DOWN = {"head_yaw_joint": 0.0, "head_pitch_joint": 0.5235}


@dataclass
class Cell:
    params: dict

    @classmethod
    def load(cls, path=TASK_CONFIG):
        document = yaml.safe_load(Path(path).read_text())
        return cls(document["pick_place_task"]["ros__parameters"])

    def __getitem__(self, dotted):
        value = self.params
        for key in dotted.split("."):
            value = value[key]
        return value

    # -- stations ----------------------------------------------------------

    def station_xy(self, station):
        return tuple(self[f"station.{station}_xy"])

    def tag_id(self, station):
        return self[f"tag.{station}_id"]

    @property
    def bench_top(self):
        return self["bench.top_z"]

    def part_position(self, station):
        """Where the workpiece rests, in the odom frame."""
        x, y = self.station_xy(station)
        return (x, y, self.bench_top + self["workpiece.size"][2] / 2.0)

    def tag_position(self, station):
        """Ground truth for the marker centre. Mirrors CellLayout.tag_pose,
        including the per-station offset override."""
        x, y = self.station_xy(station)
        offset_x, offset_y = self.params["tag"].get(
            f"{station}_to_part_xy", self["tag.to_part_xy"]
        )
        return (x - offset_x, y - offset_y, self.bench_top + self["tag.thickness"] / 2.0)

    def bench_centre(self, station):
        x, y = self.station_xy(station)
        return (x + self["bench.size_xy"][0] / 2.0 - self["bench.part_inset"], y)

    def dock_pose(self, station):
        """Where the chassis parks. Mirrors CellLayout.dock_pose at yaw 0.

        ``home`` is the taught standby pose - the wall corner - read from the
        same dock book the skills use, so Isaac's robot wakes up exactly where
        the ROS base will place itself and the recording never shows a jump."""
        import math

        if station == "home":
            import yaml as _yaml

            docks = _yaml.safe_load(
                (REPO / "Rviz" / "src" / "franzi_pick_place" / "config"
                 / "dock_poses.yaml").read_text()
            )["docks"]["home"]
            return (float(docks["x"]), float(docks["y"]), float(docks["yaw"]))
        x, y = self.station_xy(station)
        yaw = self["dock.yaw"]
        offset_x, offset_y = self["dock.offset"]
        return (
            x - (math.cos(yaw) * offset_x - math.sin(yaw) * offset_y),
            y - (math.sin(yaw) * offset_x + math.cos(yaw) * offset_y),
            yaw,
        )

    # -- bench geometry ----------------------------------------------------

    @property
    def slab_size(self):
        sx, sy = self["bench.size_xy"]
        return (sx, sy, self["bench.thickness"])

    def slab_position(self, station):
        cx, cy = self.bench_centre(station)
        return (cx, cy, self.bench_top - self["bench.thickness"] / 2.0)

    @property
    def leg_size(self):
        leg = self["bench.leg_size"]
        top = self.bench_top - self["bench.thickness"]
        return (leg, leg, top - self["ground_z"])

    def leg_positions(self, station):
        cx, cy = self.bench_centre(station)
        sx, sy = self["bench.size_xy"]
        leg = self["bench.leg_size"]
        inset = self["bench.leg_inset"]
        top = self.bench_top - self["bench.thickness"]
        height = top - self["ground_z"]
        return [
            (
                cx + dx * (sx / 2.0 - inset - leg / 2.0),
                cy + dy * (sy / 2.0 - inset - leg / 2.0),
                top - height / 2.0,
            )
            for dx in (-1.0, 1.0)
            for dy in (-1.0, 1.0)
        ]
