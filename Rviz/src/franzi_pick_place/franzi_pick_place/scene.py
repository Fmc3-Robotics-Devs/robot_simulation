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


def tag_id(station):
    return f"apriltag_{station}"


def tag_code_id(station):
    return f"apriltag_{station}_code"


# A 6x6 payload, drawn only so the marker reads as a tag in RViz rather than as
# a black slab. Detection is simulated geometrically, so the pattern carries no
# information - do not print this and expect a decoder to like it.
TAG_PAYLOAD = (
    (1, 1, 0, 1, 0, 0),
    (0, 1, 1, 0, 1, 1),
    (1, 0, 1, 1, 0, 1),
    (1, 1, 0, 0, 1, 0),
    (0, 1, 1, 0, 1, 1),
    (1, 0, 0, 1, 1, 0),
)


COLORS = {
    GROUND: (0.35, 0.35, 0.38, 1.0),
    bench_id(FEEDER): (0.72, 0.60, 0.44, 1.0),
    bench_id(MACHINE): (0.62, 0.55, 0.45, 1.0),
    bench_id(OUTFEED): (0.72, 0.60, 0.44, 1.0),
    FIXTURE: (0.35, 0.42, 0.55, 1.0),
    WORKPIECE: (0.90, 0.45, 0.15, 1.0),
    **{tag_id(station): (0.95, 0.95, 0.95, 1.0) for station in (FEEDER, MACHINE, OUTFEED)},
    **{tag_code_id(station): (0.04, 0.04, 0.04, 1.0) for station in (FEEDER, MACHINE, OUTFEED)},
}


@dataclass
class CellLayout:
    frame_id: str
    ground_z: float
    # World positions are the simulated truth: they build the world and feed the
    # mock detector. The task is not allowed to read them - it works from what
    # the camera reports about the tag.
    station_xy: dict
    tag_ids: dict
    tag_size: float
    tag_thickness: float
    tag_to_part_xy: tuple
    # Stations whose tag cannot sit on the default spot (the machine's centre
    # is under the spindle) override the surveyed offset here.
    tag_to_part_overrides: dict
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

    @property
    def tag_z(self):
        return self.bench_top_z + self.tag_thickness / 2.0

    def tag_to_part(self, station):
        return self.tag_to_part_overrides.get(station, self.tag_to_part_xy)

    def tag_pose(self, station):
        """Ground truth: the tag lies flat on the bench, facing up."""
        x, y = self.station_xy[station]
        offset = self.tag_to_part(station)
        return make_pose((x - offset[0], y - offset[1], self.tag_z))

    @property
    def part_offset_in_tag(self):
        """Where the part sits relative to the tag - the surveyed constant.

        On hardware this is the number you measure once per bench design; it is
        what turns a tag detection into a grasp pose.
        """
        return (
            self.tag_to_part_xy[0],
            self.tag_to_part_xy[1],
            self.workpiece_centre_z - self.tag_z,
        )

    def part_offset_in_tag_for(self, station):
        """Per-station surveyed constant; stations without an override use
        the shared one."""
        offset = self.tag_to_part(station)
        return (offset[0], offset[1], self.workpiece_centre_z - self.tag_z)

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


def build_bench(layout, station, part_pose):
    """A bench, positioned by where the robot believes the part rests.

    Everything is built relative to that anchor rather than to a world pose:
    with SLAM there is no world pose to build against, and anchoring the
    furniture to the same observation as the part keeps the relative geometry
    right even when the observation itself is off.
    """
    sx, sy = layout.bench_size_xy
    thickness = layout.bench_thickness
    leg = layout.leg_size
    inset = layout.leg_inset

    top = -layout.workpiece_size[2] / 2.0
    centre_x = sx / 2.0 - layout.bench_part_inset

    builder = CollisionObjectBuilder(bench_id(station), layout.frame_id)
    builder.set_pose(part_pose)
    builder.add_box((sx, sy, thickness), (centre_x, 0.0, top - thickness / 2.0))

    leg_top = top - thickness
    leg_height = layout.bench_top_z - layout.bench_thickness - layout.ground_z
    for dx in (-1.0, 1.0):
        for dy in (-1.0, 1.0):
            builder.add_box(
                (leg, leg, leg_height),
                (
                    centre_x + dx * (sx / 2.0 - inset - leg / 2.0),
                    dy * (sy / 2.0 - inset - leg / 2.0),
                    leg_top - leg_height / 2.0,
                ),
            )
    return builder.build()


def build_fixture(layout, part_pose):
    """Four walls forming the pocket, around the believed part position."""
    opening = layout.workpiece_size[0] + 2.0 * layout.pocket_clearance
    thickness = layout.pocket_wall_thickness
    height = layout.pocket_wall_height
    z = -layout.workpiece_size[2] / 2.0 + height / 2.0
    span = opening + 2.0 * thickness
    arm = opening / 2.0 + thickness / 2.0

    builder = CollisionObjectBuilder(FIXTURE, layout.frame_id)
    builder.set_pose(part_pose)
    return (
        builder.add_box((thickness, span, height), (arm, 0.0, z))
        .add_box((thickness, span, height), (-arm, 0.0, z))
        .add_box((opening, thickness, height), (0.0, arm, z))
        .add_box((opening, thickness, height), (0.0, -arm, z))
        .build()
    )


def build_workpiece(layout):
    return (
        CollisionObjectBuilder(WORKPIECE, layout.frame_id)
        .add_box(layout.workpiece_size, (0.0, 0.0, 0.0))
        .build()
    )


def _tag_centre(layout, station):
    offset = layout.tag_to_part(station)
    return (
        -offset[0],
        -offset[1],
        -layout.workpiece_size[2] / 2.0 + layout.tag_thickness / 2.0,
    )


def build_tag(layout, station, part_pose):
    """The white backing plate. This is the collision volume for the marker."""
    builder = CollisionObjectBuilder(tag_id(station), layout.frame_id)
    builder.set_pose(part_pose)
    return builder.add_box(
        (layout.tag_size, layout.tag_size, layout.tag_thickness),
        _tag_centre(layout, station),
    ).build()


def build_tag_code(layout, station, part_pose):
    """The black pattern, as a second object purely so it can be a second colour.

    A planning-scene object carries one colour for all of its primitives, which
    is why the marker cannot be drawn as a single textured plate.
    """
    size = layout.tag_size
    cell = size / 8.0
    cx, cy, cz = _tag_centre(layout, station)
    # Sits a hair above the plate so it is not z-fighting with it.
    z = cz + layout.tag_thickness
    height = layout.tag_thickness / 2.0

    builder = CollisionObjectBuilder(tag_code_id(station), layout.frame_id)
    builder.set_pose(part_pose)

    # Black border, one cell wide, on all four sides.
    for dx, dy, sx, sy in (
        (-3.5 * cell, 0.0, cell, 8.0 * cell),
        (3.5 * cell, 0.0, cell, 8.0 * cell),
        (0.0, -3.5 * cell, 6.0 * cell, cell),
        (0.0, 3.5 * cell, 6.0 * cell, cell),
    ):
        builder.add_box((sx, sy, height), (cx + dx, cy + dy, z))

    for row, cells in enumerate(TAG_PAYLOAD):
        for column, filled in enumerate(cells):
            if not filled:
                continue
            builder.add_box(
                (cell, cell, height),
                (
                    cx + (column - 2.5) * cell,
                    cy + (row - 2.5) * cell,
                    z,
                ),
            )
    return builder.build()


def build_station(layout, station, part_pose):
    """What the robot believes stands at a station, given one tag detection."""
    objects = [
        build_bench(layout, station, part_pose),
        build_tag(layout, station, part_pose),
        build_tag_code(layout, station, part_pose),
    ]
    if station == MACHINE:
        objects.append(build_fixture(layout, part_pose))
    return objects
