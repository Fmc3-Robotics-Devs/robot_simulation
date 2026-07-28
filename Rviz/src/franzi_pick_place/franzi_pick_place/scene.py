"""Geometry of the machine-tending cell.

The cell is deliberately made of boxes only: it is fast to collision-check and
every dimension is a parameter, so the layout can be retuned from
``config/task.yaml`` without touching code.
"""

from dataclasses import dataclass

from .geometry import make_pose
from .planning_scene import CollisionObjectBuilder

GROUND = "ground"
WORK_TABLE = "work_table"
FIXTURE = "machine_fixture"
WORKPIECE = "workpiece"

COLORS = {
    GROUND: (0.35, 0.35, 0.38, 1.0),
    WORK_TABLE: (0.72, 0.60, 0.44, 1.0),
    FIXTURE: (0.35, 0.42, 0.55, 1.0),
    WORKPIECE: (0.90, 0.45, 0.15, 1.0),
}


@dataclass
class CellLayout:
    frame_id: str
    ground_z: float
    table_center_xy: tuple
    table_size_xy: tuple
    table_top_z: float
    table_thickness: float
    leg_size: float
    leg_inset: float
    workpiece_size: tuple
    pick_xy: tuple
    pocket_xy: tuple
    pocket_clearance: float
    pocket_wall_thickness: float
    pocket_wall_height: float
    unload_xy: tuple

    @property
    def workpiece_centre_z(self):
        """Height of the workpiece centre when it rests on the table."""
        return self.table_top_z + self.workpiece_size[2] / 2.0

    def resting_pose(self, xy):
        return make_pose((xy[0], xy[1], self.workpiece_centre_z))

    @property
    def pick_pose(self):
        return self.resting_pose(self.pick_xy)

    @property
    def place_pose(self):
        return self.resting_pose(self.pocket_xy)

    @property
    def unload_pose(self):
        return self.resting_pose(self.unload_xy)


def build_ground(layout):
    """A floor slab, kept just below the wheel contact point.

    The 5 mm gap keeps the standing robot out of collision with its own floor
    while still blocking any plan that would dive under the table.
    """
    thickness = 0.05
    top = layout.ground_z - 0.005
    return (
        CollisionObjectBuilder(GROUND, layout.frame_id)
        .add_box((6.0, 6.0, thickness), (0.0, 0.0, top - thickness / 2.0))
        .build()
    )


def build_table(layout):
    cx, cy = layout.table_center_xy
    sx, sy = layout.table_size_xy
    top = layout.table_top_z
    thickness = layout.table_thickness
    leg = layout.leg_size
    inset = layout.leg_inset

    builder = CollisionObjectBuilder(WORK_TABLE, layout.frame_id)
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
    cx, cy = layout.pocket_xy
    opening = layout.workpiece_size[0] + 2.0 * layout.pocket_clearance
    thickness = layout.pocket_wall_thickness
    height = layout.pocket_wall_height
    z = layout.table_top_z + height / 2.0
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
    """Return every static collision object plus the workpiece at its pick pose."""
    workpiece = build_workpiece(layout)
    workpiece.pose = layout.pick_pose
    return [build_ground(layout), build_table(layout), build_fixture(layout), workpiece]
