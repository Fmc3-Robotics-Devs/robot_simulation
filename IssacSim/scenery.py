"""Shared scenery for every Isaac entry point: lights, benches, stock, tags.

One builder, three callers (ros2_cell, render_cell, showcase), so the cell the
cameras publish is the cell the stills show. Layout numbers all come from
task.yaml through `Cell`; what lives here is only appearance.

The pick target stays at `/World/workpiece` with a translate op - ros2_cell's
mirror moves that prim from `workpiece/pose`, so the path and op must not
change. The rest of the stock on the feeder bench is static dressing, kept
clear of the tag lane (the corridor the head camera reads) and of the grasp
approach above the target.
"""

import math

TEXTURES = None  # set by build() from the caller's texture directory

# Aluminium stock like the shop photo: matte-brushed, on dark bench tops.
DARK_TOP = ((0.10, 0.10, 0.11), 0.55, 0.0)
LEG_GREY = ((0.45, 0.47, 0.51), 0.5, 0.0)
# Mostly-dielectric silver: full metallic under an indoor dome just renders
# black, there is nothing bright for the metal to reflect.
ALU_BLOCK = ((0.76, 0.77, 0.79), 0.28, 0.45)

# Where the spare blocks sit relative to the pick target. dy = 0 is kept free:
# that lane holds the tag and the camera's line of sight to it.
STOCK_OFFSETS = [
    (dx, dy)
    for dx in (0.0, 0.15, 0.30)
    for dy in (-0.36, -0.18, 0.18, 0.36)
] + [(0.30, 0.0), (0.45, -0.18), (0.45, 0.18)]


def _surface(sim_utils, spec):
    rgb, roughness, metallic = spec
    return sim_utils.PreviewSurfaceCfg(
        diffuse_color=rgb, roughness=roughness, metallic=metallic
    )


def spawn_lights(sim_utils, cell):
    """Indoor hall lighting: a soft dome, ceiling fixtures over each station
    and over the aisle the robot drives, and a lamp inside the machine so the
    door is not a black hole. The old distant 'sun' is gone - it read as
    outdoors and cast one hard shadow."""
    dome = sim_utils.DomeLightCfg(intensity=900.0, color=(0.93, 0.94, 0.98))
    dome.func("/World/dome", dome)
    stations = [cell.station_xy(name) for name in ("feeder", "machine", "outfeed")]
    fixtures = [(x - 1.2, y) for x, y in stations] + [(0.0, -2.0), (0.0, 2.0)]
    for index, (x, y) in enumerate(fixtures):
        lamp = sim_utils.DiskLightCfg(
            radius=0.35, intensity=40000.0, color=(0.98, 0.96, 0.90)
        )
        lamp.func(f"/World/lights/ceiling_{index}", lamp, translation=(x, y, 3.4))
    mx, my = cell.station_xy("machine")
    glow = sim_utils.SphereLightCfg(radius=0.06, intensity=9000.0, color=(0.97, 0.97, 1.0))
    glow.func("/World/lights/machine", glow, translation=(mx + 0.45, my, 1.75))


def spawn_bench(stage, sim_utils, cell, station):
    top = _surface(sim_utils, DARK_TOP)
    legs = _surface(sim_utils, LEG_GREY)
    slab = sim_utils.CuboidCfg(size=cell.slab_size, visual_material=top)
    slab.func(f"/World/bench_{station}/slab", slab, translation=cell.slab_position(station))
    for index, position in enumerate(cell.leg_positions(station)):
        leg = sim_utils.CuboidCfg(size=cell.leg_size, visual_material=legs)
        leg.func(f"/World/bench_{station}/leg_{index}", leg, translation=position)


def spawn_stock(stage, sim_utils, cell):
    """The bin of raw blocks the robot chooses from, around the pick target."""
    size = tuple(cell["workpiece.size"])
    px, py, pz = cell.part_position("feeder")
    half_x, half_y = (d / 2.0 - 0.06 for d in cell["bench.size_xy"])
    centre_x, centre_y = cell.bench_centre("feeder")
    for index, (dx, dy) in enumerate(STOCK_OFFSETS):
        x, y = px + dx, py + dy
        if abs(x - centre_x) > half_x or abs(y - centre_y) > half_y:
            continue
        block = sim_utils.CuboidCfg(
            size=size, visual_material=_surface(sim_utils, ALU_BLOCK)
        )
        block.func(
            f"/World/stock/block_{index}",
            block,
            translation=(x, y, pz),
            # A hand-placed bin is never square to the bench: a fixed pseudo-
            # random yaw per slot sells it without nondeterminism.
            orientation=(
                math.cos(0.35 * math.sin(7.0 * index)),
                0.0,
                0.0,
                math.sin(0.35 * math.sin(7.0 * index)),
            ),
        )


# Free-standing shelf units dressing the hall - and, later, the obstacles a
# Nav2 stage has to steer around. Placed clear of today's straight-line drive
# segments (home -> docks -> stations), because today's base drives blind.
RACKS = [((-2.6, -2.6), 90.0), ((-2.8, 1.8), 0.0), ((0.6, 6.0), 0.0), ((4.6, -2.4), 0.0)]


CARDBOARD = ((0.72, 0.58, 0.40), 0.8, 0.0)
RACK_STEEL = ((0.30, 0.34, 0.40), 0.55, 0.3)


def spawn_rack(stage, sim_utils, name, centre, yaw_degrees):
    """A two-shelf warehouse rack with a few boxes: 1.6 x 0.6 m, 2 m tall."""
    steel = _surface(sim_utils, RACK_STEEL)
    yaw = math.radians(yaw_degrees)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

    def place(cfg, part, dx, dy, z):
        cfg.func(
            f"/World/racks/{name}/{part}",
            cfg,
            translation=(
                centre[0] + cos_yaw * dx - sin_yaw * dy,
                centre[1] + sin_yaw * dx + cos_yaw * dy,
                z,
            ),
            orientation=(math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)),
        )

    for index, (dx, dy) in enumerate(
        [(sx * 0.27, sy * 0.77) for sx in (-1, 1) for sy in (-1, 1)]
    ):
        post = sim_utils.CuboidCfg(size=(0.06, 0.06, 2.0), visual_material=steel)
        place(post, f"post_{index}", dx, dy, 0.91)
    for level, z in enumerate((0.02, 0.70, 1.35)):
        board = sim_utils.CuboidCfg(size=(0.60, 1.60, 0.04), visual_material=steel)
        place(board, f"board_{level}", 0.0, 0.0, z)
    for index, (dy, z, size) in enumerate(
        [(-0.45, 0.22, 0.36), (0.25, 0.19, 0.30), (-0.1, 0.88, 0.32), (0.5, 1.51, 0.28)]
    ):
        box = sim_utils.CuboidCfg(
            size=(size, size, size), visual_material=_surface(sim_utils, CARDBOARD)
        )
        place(box, f"box_{index}", 0.0, dy, z + size / 2.0)


def spawn_machining_center(sim_utils, cell, usd_path):
    """The real 智能制造中心 shopfloor (converted from the customer's STEP).

    The CAD is millimetres with its main aisle running along its own x; a
    quarter turn lays that aisle along the cell's y axis, and the translation
    puts the cell's three stations inside it. The surveyed station poses do
    not move - the hall contributes walls, the office, and the machine row
    (DMG, EMAG, MAG, ...) as real geometry around them, which is exactly what
    the lidar and Nav2 should be seeing instead of invented clutter.

    In world coordinates after this transform: the machine row stands west of
    x = -2.5, the nearest workstation (M14) reaches x = -0.8 just south-west
    of the robot's home, the east wall runs at x = 3.8 with its doorway at
    y in [-4.2, 1.2], and the north wall sits at y = 9.6.
    """
    cfg = sim_utils.UsdFileCfg(usd_path=str(usd_path), scale=(0.001, 0.001, 0.001))
    cfg.func(
        "/World/machining_center",
        cfg,
        translation=(6.3, -20.0, cell["ground_z"]),
        orientation=(0.7071068, 0.0, 0.0, 0.7071068),
    )


def build(stage, sim_utils, cell, textures, warehouse_usd=None, machining_center_usd=None):
    """Everything except the robot and the cameras.

    Two worlds share this entry point. The original warehouse cell dresses
    itself: hand-built engraving machine, racks, the stock pile. The CAD
    shopfloor (智能制造中心) *is* a scene already - a surveyed hall with its
    own machine row - so it gets only what the task itself requires: three
    benches, their tags, the workpiece. Nothing invented is added on top of a
    customer's real layout.
    """
    import apriltags
    import machine
    from cell import STATIONS
    from render_cell import spawn_textured_quad

    floor_z = cell["ground_z"]
    if machining_center_usd:
        # The hall brings its own floor, walls and machines.
        spawn_machining_center(sim_utils, cell, machining_center_usd)
    elif warehouse_usd:
        cfg = sim_utils.UsdFileCfg(usd_path=warehouse_usd)
        cfg.func("/World/warehouse", cfg, translation=(0.0, 0.0, floor_z))
    else:
        ground = sim_utils.GroundPlaneCfg()
        ground.func("/World/ground", ground, translation=(0.0, 0.0, floor_z))
    spawn_lights(sim_utils, cell)

    for station in STATIONS:
        if station == "machine" and not machining_center_usd:
            machine.spawn_machine(stage, sim_utils, cell)
        else:
            spawn_bench(stage, sim_utils, cell, station)
        tag_id = cell.tag_id(station)
        texture = apriltags.write_marker(tag_id, textures / f"tag_{tag_id}.png")
        # Half a turn: the umich detector (apriltag_ros) reads the pattern's
        # frame opposite to OpenCV's ArUco, and the survey follows the former.
        spawn_textured_quad(
            stage,
            f"/World/tag_{station}",
            cell["tag.size"] * apriltags.sheet_scale(),
            texture,
            cell.tag_position(station),
            yaw_degrees=180.0,
        )

    if not machining_center_usd:
        for index, (centre, yaw) in enumerate(RACKS):
            spawn_rack(stage, sim_utils, f"rack_{index}", centre, yaw)
        spawn_stock(stage, sim_utils, cell)
    workpiece = sim_utils.CuboidCfg(
        size=tuple(cell["workpiece.size"]),
        visual_material=_surface(sim_utils, ALU_BLOCK),
    )
    # The blank lies at whatever angle it happens to lie (scenery truth from
    # task.yaml); the robot's vision has to measure it, never look it up.
    spawn_yaw = math.radians(cell["workpiece.spawn_yaw_deg"])
    workpiece.func(
        "/World/workpiece",
        workpiece,
        translation=cell.part_position("feeder"),
        orientation=(math.cos(spawn_yaw / 2.0), 0.0, 0.0, math.sin(spawn_yaw / 2.0)),
    )
