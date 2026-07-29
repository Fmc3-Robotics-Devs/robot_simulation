"""The engraving machine at the `machine` station, modelled on the shop photos
in `scene/` (a DMG MORI-style 5-axis cell: enclosure, rotary table, zero-point
pallet with clamping chucks, self-centring vise, overhead spindle).

Everything the task touches is anchored to task.yaml, so the running system
does not notice the machine at all:

* the pallet's top plate lies exactly in the bench-top plane (`bench.top_z`),
  where the workpiece, the pocket and the AprilTag already live;
* the vise jaws reproduce the MoveIt pocket dimensions one-for-one, and the
  spindle hangs directly over them - machining happens where the part is
  loaded. The spindle bit stops 1.38 m up; the wrist tops out around 1.30 m
  during the vertical insertion, which is the clearance that matters;
* the tag sits on the plate's free front corner (`tag.machine_to_part_xy`),
  out from under the spindle, where the head camera sees it past the door;
* the enclosure keeps clear of the flight corridors: the chassis never passes
  0.28 m short of the near edge (the bench legs already proved that), and the
  carried part sweeps ~0.13 m short of it at z ~ 1.0 while strafing, so every
  panel above the plinth starts 0.10 m behind the near edge. The front door
  aperture, open to z = 1.60, is what the arm reaches through; the spindle's
  quill enters through a slot between the two header halves.

The MoveIt collision world still only knows the bench boxes and the pocket -
the enclosure is visual. Keep it out of the arm's way when changing sizes.
"""

PANEL = ((0.72, 0.73, 0.76), 0.45, 0.0)
FRAME = ((0.13, 0.14, 0.16), 0.55, 0.0)
STEEL = ((0.34, 0.35, 0.39), 0.45, 0.7)
ALU = ((0.60, 0.62, 0.66), 0.35, 0.8)


def _materials(sim_utils):
    return {
        name: sim_utils.PreviewSurfaceCfg(
            diffuse_color=rgb, roughness=rough, metallic=metal
        )
        for name, (rgb, rough, metal) in
        {"panel": PANEL, "frame": FRAME, "steel": STEEL, "alu": ALU}.items()
    }


def spawn_machine(stage, sim_utils, cell):
    """Build the machine around the station's load point. Visual only."""
    px, py = cell.station_xy("machine")  # the part / fixture centre
    top = cell.bench_top
    floor = cell["ground_z"]
    # Anchors that survive bench resizing and station moves: the near edge is
    # pinned to the part inset (the robot-facing side never moves relative to
    # the dock), the far side follows the bench size.
    near = px - cell["bench.part_inset"]
    far = near + cell["bench.size_xy"][0]
    front = near + 0.10  # no enclosure panel in front of this above the plinth
    mid = near + 0.30  # table stack centre
    rear = far + 0.35  # inner face of the back wall
    mat = _materials(sim_utils)

    def box(name, centre, size, look):
        cfg = sim_utils.CuboidCfg(size=size, visual_material=mat[look])
        cfg.func(f"/World/machine/{name}", cfg, translation=centre)

    def disc(name, centre, radius, height, look):
        cfg = sim_utils.CylinderCfg(
            radius=radius, height=height, visual_material=mat[look]
        )
        cfg.func(f"/World/machine/{name}", cfg, translation=centre)

    def slab_z(top_z, height):
        return top_z - height / 2.0

    # -- table stack, floor to the bench-top plane -------------------------
    box("plinth", ((near + rear) / 2.0, py, slab_z(0.54, 0.54 - floor)),
        (rear - near, 1.10, 0.54 - floor), "frame")
    disc("table", (mid, py, slab_z(0.61, 0.07)), 0.34, 0.07, "steel")
    disc("drum", (mid, py, slab_z(0.70, 0.09)), 0.29, 0.09, "frame")
    box("pallet", (mid, py, slab_z(0.735, 0.03)), (0.44, 0.38, 0.03), "frame")
    for index, (dx, dy) in enumerate(((-0.16, 0.13), (-0.16, -0.13), (0.14, 0.0))):
        disc(f"chuck_{index}", (mid + dx, py + dy, slab_z(0.768, 0.032)), 0.055, 0.032, "steel")
    # Wide enough in y that the corner tag (tag.machine_to_part_xy) lies on it.
    box("plate", (near + 0.30, py, slab_z(top, 0.03)), (0.60, 0.64, 0.03), "alu")

    # -- vise: the MoveIt pocket, rendered in hardened steel ---------------
    half = cell["workpiece.size"][0] / 2.0 + cell["pocket.clearance"]
    wall = cell["pocket.wall_thickness"]
    height = cell["pocket.wall_height"]
    length = 2.0 * half + 2.0 * wall
    for axis in ("x", "y"):
        for side in (-1.0, 1.0):
            offset = side * (half + wall / 2.0)
            centre = (px + offset, py, 0) if axis == "x" else (px, py + offset, 0)
            size = (wall, length, height) if axis == "x" else (length, wall, height)
            box(f"jaw_{axis}{'p' if side > 0 else 'n'}",
                (centre[0], centre[1], top + height / 2.0), size, "frame")

    # -- spindle straight over the fixture, bit high above the arm's path --
    ax = px + 0.02  # housing axis; the flanks stay behind the front plane
    disc("bit", (px + 0.01, py, slab_z(1.46, 0.08)), 0.004, 0.08, "steel")
    disc("holder", (ax, py, slab_z(1.54, 0.08)), 0.028, 0.08, "steel")
    disc("spindle", (ax, py, slab_z(1.74, 0.20)), 0.045, 0.20, "frame")
    box("quill", ((px - 0.04 + near + 0.44) / 2.0, py, slab_z(1.88, 0.14)),
        (near + 0.44 - px + 0.04, 0.12, 0.14), "panel")
    box("head", ((near + 0.44 + rear - 0.28) / 2.0, py, slab_z(1.90, 0.30)),
        (rear - 0.28 - near - 0.44, 0.30, 0.30), "panel")
    box("column", (rear - 0.155, py, slab_z(1.95, 1.41)), (0.25, 0.32, 1.41), "panel")

    # -- enclosure, open at the front door ---------------------------------
    # White-on-white reads as a solid slab under the dome light, so the deep
    # surfaces (back wall, headers, skirts, aperture trim) are dark: the door
    # opening has to be visible as an opening from the robot's side. Panels
    # butt against each other instead of interpenetrating - overlapping boxes
    # put near-coplanar faces in the same place and the RTX denoiser turns
    # that into a sparkling seam.
    depth = rear + 0.05 - front
    depth_mid = front + depth / 2.0
    tall = 2.02 - floor
    for side in (-1.0, 1.0):
        tag = "p" if side > 0 else "n"
        box(f"side_{tag}",
            (depth_mid, py + side * 0.855, slab_z(2.02, tall)), (depth, 0.05, tall), "panel")
        box(f"skirt_{tag}",
            (depth_mid, py + side * 0.9075, slab_z(0.55, 0.55 - floor)), (depth, 0.055, 0.55 - floor), "frame")
        box(f"post_{tag}",
            (front + 0.025, py + side * 0.665, slab_z(1.60, 1.60 - floor)), (0.05, 0.33, 1.60 - floor), "panel")
        box(f"trim_{tag}",
            (front + 0.02, py + side * 0.475, slab_z(1.60, 1.60 - 0.54)), (0.06, 0.05, 1.60 - 0.54), "frame")
        # The header is split so the spindle quill passes between the halves.
        box(f"header_{tag}",
            (front + 0.025, py + side * 0.4725, slab_z(2.02, 0.42)), (0.05, 0.715, 0.42), "frame")
    box("back", (rear + 0.025, py, slab_z(2.02, tall)), (0.05, 1.66, tall), "frame")
    box("roof", (depth_mid, py, slab_z(2.08, 0.06)), (depth, 1.80, 0.06), "frame")

    # -- dressing: signal tower and the maker's mark -----------------------
    disc("mast", (front + 0.04, py + 0.78, slab_z(2.20, 0.12)), 0.012, 0.12, "frame")
    for index, colour in enumerate(((0.85, 0.1, 0.1), (0.9, 0.6, 0.1), (0.1, 0.7, 0.2))):
        lamp = sim_utils.CylinderCfg(
            radius=0.030,
            height=0.05,
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=colour, emissive_color=colour, roughness=0.4
            ),
        )
        lamp.func(
            f"/World/machine/lamp_{index}",
            lamp,
            translation=(front + 0.04, py + 0.78, 2.225 + index * 0.05),
        )

    import paint
    from render_cell import spawn_textured_quad
    from pxr import UsdGeom

    quad = spawn_textured_quad(
        stage, "/World/machine/logo", 0.30, paint.write_logo(),
        (front - 0.005, py + 0.4725, 1.81), yaw_degrees=-90.0,
    )
    UsdGeom.Xformable(quad).AddRotateXOp().Set(90.0)
