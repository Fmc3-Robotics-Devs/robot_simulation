"""Geometry of the machine-tending cell.

Three benches in a row, far enough apart that the chassis has to drive between
them. Everything is boxes: fast to collision-check, and every dimension is a
parameter so the cell can be retuned from ``config/task.yaml``.

Two frames matter here. Stations are world poses. ``dock_offset`` is where a
station has to end up *in the base frame* once the robot has parked, and it is
the one number that ties the two together - it must land inside the arm's
reachable envelope (see the ``reach_map`` tool).
"""

import math
from dataclasses import dataclass

from .geometry import make_pose
from .planning_scene import CollisionObjectBuilder

GROUND = "ground"
FIXTURE = "machine_fixture"
WORKPIECE = "workpiece"
FEEDER = "feeder"
MACHINE = "machine"
OUTFEED = "outfeed"

STATIONS = (FEEDER, MACHINE, OUTFEED)


def bench_id(station):
    return f"bench_{station}"


COLORS = {
    GROUND: (0.35, 0.35, 0.38, 1.0),
    bench_id(FEEDER): (0.72, 0.60, 0.44, 1.0),
    bench_id(MACHINE): (0.62, 0.55, 0.45, 1.0),
    bench_id(OUTFEED): (0.72, 0.60, 0.44, 1.0),
    FIXTURE: (0.35, 0.42, 0.55, 1.0),
    WORKPIECE: (0.90, 0.45, 0.15, 1.0),
}


@dataclass
class CellLayout:
    frame_id: str
    ground_z: float
    station_xy: dict
    bench_size_xy: tuple
    bench_top_z: float
    bench_thickness: float
    bench_part_inset: float
    leg_size: float
    leg_inset: float
    workpiece_size: tuple
    pocket_clearance: float
    pocket_wall_thickness: float
    pocket_wall_height: float
    dock_offset: tuple
    dock_yaw: float
    standby_retreat: float

    @property
    def workpiece_centre_z(self):
        """Height of the workpiece centre when it rests on a bench."""
        return self.bench_top_z + self.workpiece_size[2] / 2.0

    def part_pose(self, station):
        x, y = self.station_xy[station]
        return make_pose((x, y, self.workpiece_centre_z))

    def bench_centre(self, station):
        """Benches sit back from their part, which rests near the near edge."""
        x, y = self.station_xy[station]
        return (x + self.bench_size_xy[0] / 2.0 - self.bench_part_inset, y)

    def dock_pose(self, station):
        """World pose the chassis parks at to work a station.

        Inverts ``station = base + R(yaw) * dock_offset``.
        """
        x, y = self.station_xy[station]
        yaw = self.dock_yaw
        offset_x, offset_y = self.dock_offset
        return (
            x - (math.cos(yaw) * offset_x - math.sin(yaw) * offset_y),
            y - (math.sin(yaw) * offset_x + math.cos(yaw) * offset_y),
            yaw,
        )

    def standby_pose(self):
        """Backed off from the machine so the cycle can run."""
        x, y, yaw = self.dock_pose(MACHINE)
        return (x - math.cos(yaw) * self.standby_retreat,
                y - math.sin(yaw) * self.standby_retreat,
                yaw)


def build_ground(layout):
    """A floor slab, kept just below the wheel contact point.

    The 5 mm gap keeps the standing robot out of collision with its own floor
    while still blocking any plan that would dive under a bench.
    """
    thickness = 0.05
    top = layout.ground_z - 0.005
    return (
        CollisionObjectBuilder(GROUND, layout.frame_id)
        .add_box((14.0, 14.0, thickness), (0.0, 0.0, top - thickness / 2.0))
        .build()
    )


def build_bench(layout, station):
    cx, cy = layout.bench_centre(station)
    sx, sy = layout.bench_size_xy
    top = layout.bench_top_z
    thickness = layout.bench_thickness
    leg = layout.leg_size
    inset = layout.leg_inset

    builder = CollisionObjectBuilder(bench_id(station), layout.frame_id)
    builder.add_box((sx, sy, thickness), (cx, cy, top - thickness / 2.0))

    leg_top = top - thickness
    leg_height = leg_top - layout.ground_z
    for dx in (-1.0, 1.0):
        for dy in (-1.0, 1.0):
            builder.add_box(
                (leg, leg, leg_height),
                (
                    cx + dx * (sx / 2.0 - inset - leg / 2.0),
                    cy + dy * (sy / 2.0 - inset - leg / 2.0),
                    leg_top - leg_height / 2.0,
                ),
            )
    return builder.build()


def build_fixture(layout):
    """Four walls forming the pocket the workpiece is inserted into."""
    cx, cy = layout.station_xy[MACHINE]
    opening = layout.workpiece_size[0] + 2.0 * layout.pocket_clearance
    thickness = layout.pocket_wall_thickness
    height = layout.pocket_wall_height
    z = layout.bench_top_z + height / 2.0
    span = opening + 2.0 * thickness
    arm = opening / 2.0 + thickness / 2.0

    return (
        CollisionObjectBuilder(FIXTURE, layout.frame_id)
        .add_box((thickness, span, height), (cx + arm, cy, z))
        .add_box((thickness, span, height), (cx - arm, cy, z))
        .add_box((opening, thickness, height), (cx, cy + arm, z))
        .add_box((opening, thickness, height), (cx, cy - arm, z))
        .build()
    )


def build_workpiece(layout):
    return (
        CollisionObjectBuilder(WORKPIECE, layout.frame_id)
        .add_box(layout.workpiece_size, (0.0, 0.0, 0.0))
        .build()
    )


def build_cell(layout):
    """Every static collision object plus the workpiece on the feeder bench."""
    workpiece = build_workpiece(layout)
    workpiece.pose = layout.part_pose(FEEDER)
    return (
        [build_ground(layout)]
        + [build_bench(layout, station) for station in STATIONS]
        + [build_fixture(layout), workpiece]
    )
