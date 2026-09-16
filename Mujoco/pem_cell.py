#!/usr/bin/env python3
"""Build the PEM stacking cell scene around the robot: model/pem_scene.xml.

The layout is ``pem_cell.yaml``, which carries the numbers of "PEM
Project/20260906 Measurements.pdf" and the electrode sheet drawings; nothing
here hardcodes a position. The scene has

* the frame: 50 x 50 aluminium profile, a base beam on the floor and a post
  under each tray;
* the anode and cathode trays (``tray.stl``: a base plate with four L-shaped
  corner guides) and the assembly tray (a plain 205 x 192 plate);
* a side table left of the anode post with stacks of electrode sheets on it,
  anode and cathode separate, each in a fork carrier that opens and closes:
  a tine under the stack, a pad clamping it from above (the carrier's own
  actuator ``<carrier>_clamp``), a handle for the gripper on top;
* named sites for scripts: ``<tray>`` at each tray's floor centre,
  ``<carrier>_handle`` on top of each handle, and ``<station>_dock`` where the
  robot parks to work at a tray or a carrier.

What has to be got right, and why:

* **Tray collision is not the mesh's hull.** The hull of ``tray.stl`` is a
  solid 205 x 137 x 45 block - nothing would go in. The mesh is drawn and
  seen by ray casts (the lidars), but collides as boxes measured off it: the
  base plate and the eight guide walls. The plate's top is a shallow roof
  (3 mm at the edges, 7 mm over the middle 19 mm); a rigid stack cannot
  follow a 2.5 deg roof, so it collides flat at the 7 mm crown. The walls
  stand 0.25 mm back from the mesh: its pocket is exactly the anode sheet's
  119 mm, a press fit for a rigid stack.
* **A stack is one rigid body.** Body and tab of the stacked sheets are two
  boxes, with the sheets' drawn outline and thickness times the sheet count.
  The tabs are bare foil: the tab stack is drawn at full height (a sheaf of
  foils looks like a block) but weighs only the foil.
* **The carrier fits the tray.** The stack is 119 mm wide and cannot pass the
  corner guides, so it goes into a tray from above; only the carrier's tine,
  pad and arm (40 mm wide) pass the 47 mm gap between the guides at the
  tray's short ends, which is how the empty carrier leaves along x. The spine
  stands just beyond the tray's end when the stack is centred over it.

Run from the repository root after convert_urdf.py (it includes
model/franzi.xml):

    Mujoco/.venv/bin/python Mujoco/pem_cell.py
"""

import argparse
import math
import os
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import convert_urdf  # noqa: E402
from convert_urdf import fmt, look_at, placement, rpy_matrix  # noqa: E402

CONFIG = HERE / "pem_cell.yaml"
OUT = HERE / "model" / "pem_scene.xml"
MM = 1e-3

# tray.stl, in its own frame (mm, corner at the origin, 205 x 137 x 45),
# measured off the mesh. Guide walls at the (0, 0) corner as (x0, x1, y0, y1);
# the other corners mirror them. Their inner faces stand 11 mm from the short
# edges and 9 mm from the long ones: a 183 x 119 pocket, the anode sheet's
# 182 x 119 body.
TRAY_SIZE = (205.0, 137.0, 45.0)
TRAY_CROWN = 7.0
TRAY_GUIDES = ((0, 7, 0, 45), (0, 11, 30, 45), (0, 20, 0, 5), (16, 20, 0, 9))
# The collision guides stand this much (mm) back from the mesh's inner faces.
# The drawn pocket is the anode sheet's size exactly - fine for a punched,
# flexible sheet, a press fit for a rigid stack: a 0.06 deg yaw already jams it.
TRAY_PLAY = 0.25

MATERIALS = {
    "profile": dict(rgba="0.72 0.74 0.77 1", specular="0.6", shininess="0.5"),
    "slot": dict(rgba="0.2 0.21 0.23 1"),
    "tray": dict(rgba="0.2 0.21 0.23 1", specular="0.3", shininess="0.3"),
    "assembly_tray": dict(rgba="0.42 0.44 0.46 1", specular="0.3", shininess="0.3"),
    "table": dict(rgba="0.55 0.5 0.42 1", specular="0.1", shininess="0.1"),
    "table_frame": dict(rgba="0.25 0.27 0.3 1"),
    "anode_coating": dict(rgba="0.1 0.1 0.11 1", specular="0.2", shininess="0.2"),
    "anode_tab": dict(rgba="0.78 0.45 0.25 1", specular="0.8", shininess="0.8"),
    "cathode_coating": dict(rgba="0.2 0.2 0.21 1", specular="0.2", shininess="0.2"),
    "cathode_tab": dict(rgba="0.8 0.81 0.83 1", specular="0.8", shininess="0.8"),
    "carrier": dict(rgba="0.2 0.42 0.72 1", specular="0.5", shininess="0.5"),
    "tine": dict(rgba="0.62 0.64 0.67 1", specular="0.9", shininess="0.9"),
}
ALUMINIUM, STEEL = 2700, 7850
CLAMP_CLOSED = -0.010  # pad target 10 mm into the stack: presses at the force limit
# Carrier and stack contacts: stiffer than MuJoCo's default (0.02 s), which
# lets a 40 N clamp sink the pad ~2 mm into the stack.
STIFF = dict(solref="0.004 1")
# The tine has to slide out from under a stack: a low-friction (PTFE-coated)
# face, which its priority makes the contact use against the stack and tray.
TINE = dict(friction="0.15 0.005 0.0001", priority="2")
COLLISION_RGBA = "0.9 0.4 0.2 0.4"


class Cell:
    """pem_cell.yaml turned into positions in the cell frame (metres)."""

    def __init__(self, config):
        self.config = config
        t = config["trays"]
        self.profile = config["profile"] * MM
        self.tray_size = np.array(TRAY_SIZE) * MM
        width, depth = self.tray_size[:2]
        half_gap = t["inner_gap_anode_cathode"] * MM / 2
        # The gaps are exact; the assembly tray takes what is left between them.
        left = -half_gap + t["inner_gap_anode_assembly"] * MM
        right = half_gap - t["inner_gap_assembly_cathode"] * MM
        aw, ad = right - left, t["assembly_depth"] * MM
        self.trays = {  # name: (centre x, centre y, underside z, size x, size y)
            "anode_tray": (-half_gap - width / 2, 0.0, t["height_anode"] * MM, width, depth),
            "cathode_tray": (half_gap + width / 2, 0.0, t["height_cathode"] * MM, width, depth),
            "assembly_tray": ((left + right) / 2, t["assembly_centre_offset"] * MM,
                              t["height_assembly"] * MM, aw, ad),
        }

        tb = config["table"]
        sx, sy = np.array(tb["size"]) * MM
        anode_post_outer = self.trays["anode_tray"][0] - self.profile / 2
        right = anode_post_outer - tb["gap"] * MM
        self.table = (right - sx / 2, tb["front"] * MM + sy / 2, tb["height"] * MM, sx, sy)

        # Carriers along the table, cathodes left, anodes right (nearer the
        # anode tray): (carrier, stack, kind, stack centre x, y).
        p = config["stacks"]
        kinds = ["cathode"] * p["count"]["cathode"] + ["anode"] * p["count"]["anode"]
        self.carriers = []
        numbers = {"anode": 0, "cathode": 0}
        for i, kind in enumerate(kinds):
            numbers[kind] += 1
            body_depth = config["sheets"][kind]["body"][1] * MM
            x = self.table[0] + p["pitch"] * MM * (i - (len(kinds) - 1) / 2)
            y = tb["front"] * MM + p["front_margin"] * MM + body_depth / 2
            n = numbers[kind]
            self.carriers.append((f"{kind}_carrier_{n}", f"{kind}_stack_{n}", kind, x, y))

    def sheet(self, kind):
        """Body (x, y), tab (x, y), tab centre offset, stack height (m)."""
        s = self.config["sheets"][kind]
        body, tab = np.array(s["body"]) * MM, np.array(s["tab"]) * MM
        height = s["thickness"] * self.config["stacks"]["sheets"] * MM
        tab_centre = np.array([-body[0] / 2 + s["tab_offset"] * MM + tab[0] / 2,
                               body[1] / 2 + tab[1] / 2])
        return body, tab, tab_centre, height

    def stack_mass(self, kind):
        """(mass, centre of mass) of a stack in the carrier frame."""
        s = self.config["sheets"][kind]
        body, tab, tab_centre, height = self.sheet(kind)
        masses = [s["density"] * body[0] * body[1] * height,
                  s["tab_density"] * tab[0] * tab[1] * height]
        centres = [np.array([0, 0, height / 2]), np.array([*tab_centre, height / 2])]
        return sum(masses), sum(mm * c for mm, c in zip(masses, centres)) / sum(masses)

    def carrier(self, kind):
        """Carrier parts for a ``kind`` stack, in the carrier frame: origin
        at the stack's body centre on the tine's top, x along the stack
        towards the spine. {part: (size, centre, density)} for the carrier
        body and for its clamp (whose frame sits on the stack's top), plus
        heights. The handle stands over the loaded carrier's centre of mass,
        so a held carrier hangs level."""
        c = self.config["carrier"]
        body, _, _, height = self.sheet(kind)
        w, tine, spine = c["width"] * MM, c["tine"] * MM, c["spine"] * MM
        inner = self.tray_size[0] / 2 + c["spine_clearance"] * MM  # spine's inner face
        far = -body[0] / 2 + c["tine_margin"] * MM
        pad_len, pad_t = np.array(c["pad"]) * MM
        arm_z = height + pad_t + c["travel"] * MM + 2 * MM  # arm underside
        top = arm_z + c["arm"] * MM
        hx, hy, hz = np.array(c["handle"]) * MM
        rod = top - height - pad_t
        clamp = {
            "pad": ([pad_len, w, pad_t], [0, 0, height + pad_t / 2], ALUMINIUM),
            "rod": ([12 * MM, 12 * MM, rod], [0, 0, height + pad_t + rod / 2], ALUMINIUM),
        }
        handle_x = 0.0
        for _ in range(5):  # the arm's length depends on where the handle is
            arm_from = min(handle_x - hx / 2, -pad_len / 2)
            parts = {
                "tine": ([inner - far, w, tine], [(inner + far) / 2, 0, -tine / 2], STEEL),
                "spine": ([spine, w, top + tine], [inner + spine / 2, 0, (top - tine) / 2],
                          ALUMINIUM),
                "arm": ([inner + spine - arm_from, w, top - arm_z],
                        [(inner + spine + arm_from) / 2, 0, (arm_z + top) / 2], ALUMINIUM),
            }
            mass, com = self.stack_mass(kind)
            moment = mass * com[0]
            for size, centre, density in [*parts.values(), *clamp.values()]:
                m = density * np.prod(size)
                mass, moment = mass + m, moment + m * centre[0]
            handle_x = moment / mass  # the handle, on top, keeps this balance
        parts["handle"] = ([hx, hy, hz], [handle_x, 0, top + hz / 2], ALUMINIUM)
        for part in clamp.values():  # into the clamp's frame, on the stack's top
            part[1][2] -= height
        return {
            "parts": parts, "clamp": clamp,
            "height": height, "tine": tine, "handle": np.array([handle_x, 0, top + hz]),
            "open": c["travel"] * MM - 2 * MM,
        }

    def to_world(self, xyz):
        pl = self.config["placement"]
        rot = rpy_matrix(0, 0, math.radians(pl["yaw"]))
        return np.array([pl["x"] * MM, pl["y"] * MM, 0.0]) + rot @ np.asarray(xyz, float)

    @property
    def yaw(self):
        return math.radians(self.config["placement"]["yaw"])

    def dock(self, x, y, standoff, arm_offset):
        """World (x, y, yaw) of the base docked at cell point (x, y), mm."""
        base = self.to_world([x + arm_offset * MM, y - standoff * MM, 0.0])
        return base[0], base[1], self.yaw + math.pi / 2


def build(cell, ground_z, keyframes, tray_mesh):
    """The scene element. ``tray_mesh`` is tray.stl's path relative to the
    model's meshdir (franzi.xml sets it for every mesh)."""
    extent = 2.8
    # noslip: a carrier hangs from the fingers for a minute at a time, and
    # MuJoCo's soft friction lets it creep ~0.2 mm/s in the grip without it.
    mujoco, asset, world = convert_urdf.environment(
        "franzi_pem_cell", ground_z, center=cell.to_world([-300 * MM, -300 * MM, 0.6]),
        extent=extent, noslip_iterations="4")
    for name, attrs in MATERIALS.items():
        ET.SubElement(asset, "material", name=name, **attrs)
    ET.SubElement(asset, "mesh", name="tray", file=tray_mesh, scale=fmt([MM] * 3))
    yaw = rpy_matrix(0, 0, cell.yaw)

    def static_body(name, xyz):
        return ET.SubElement(world, "body", name=name, **placement(cell.to_world(xyz), yaw))

    def box(parent, size, pos, material, name=None, **attrs):
        attrs = {"type": "box", "size": fmt(np.asarray(size) / 2), "pos": fmt(pos),
                 "material": material, **attrs}
        if name:
            attrs["name"] = name
        return ET.SubElement(parent, "geom", attrs)

    def profile(parent, name, centre, length, axis):
        """A 50 x 50 profile along ``axis`` (0 = x, 2 = z) with its slots."""
        size = np.full(3, cell.profile)
        size[axis] = length
        box(parent, size, centre, "profile", name=name)
        slot, proud = cell.config["slot"] * MM, 0.2 * MM
        for across in (a for a in range(3) if a != axis):
            for sign in (-1, 1):
                if across == 2 and sign < 0:
                    continue  # the beam's underside is on the floor
                strip = size.copy()
                strip[across] = proud
                strip[3 - axis - across] = slot
                pos = np.array(centre, float)
                pos[across] += sign * (cell.profile / 2)
                box(parent, strip, pos, "slot", contype="0", conaffinity="0", group="2")

    # The frame.
    frame = static_body("frame", [0, 0, 0])
    p = cell.profile
    xs = [cell.trays[t][0] for t in ("anode_tray", "assembly_tray", "cathode_tray")]
    span = (xs[0] - p / 2, xs[2] + p / 2)
    profile(frame, "beam", [(span[0] + span[1]) / 2, 0, ground_z + p / 2], span[1] - span[0], 0)
    for name in ("anode_tray", "assembly_tray", "cathode_tray"):
        x, _, top, _, _ = cell.trays[name]
        bottom = ground_z + p
        profile(frame, f"{name}_post", [x, 0, (bottom + ground_z + top) / 2],
                ground_z + top - bottom, 2)

    # Anode and cathode trays: mesh drawn, boxes collide.
    tw, td, th = cell.tray_size
    crown = TRAY_CROWN * MM
    for name in ("anode_tray", "cathode_tray"):
        x, y, z, _, _ = cell.trays[name]
        body = static_body(name, [x, y, ground_z + z])
        ET.SubElement(body, "geom", type="mesh", mesh="tray", material="tray",
                      pos=fmt([-tw / 2, -td / 2, 0]), contype="0", conaffinity="0")
        collide = dict(group="3", rgba=COLLISION_RGBA)
        box(body, [tw, td, crown], [0, 0, crown / 2], "tray", name=f"{name}_base", **collide)
        for sx in (-1, 1):
            for sy in (-1, 1):
                for x0, x1, y0, y1 in TRAY_GUIDES:
                    x1, y1 = x1 - TRAY_PLAY, y1 - TRAY_PLAY
                    size = np.array([x1 - x0, y1 - y0, TRAY_SIZE[2] - TRAY_CROWN]) * MM
                    centre = np.array([sx * (tw / 2 - (x0 + x1) / 2 * MM),
                                       sy * (td / 2 - (y0 + y1) / 2 * MM),
                                       crown + size[2] / 2])
                    box(body, size, centre, "tray", **collide)
        ET.SubElement(body, "site", name=name, pos=fmt([0, 0, crown]), group="4", size="0.01")

    x, y, z, aw, ad = cell.trays["assembly_tray"]
    thick = cell.config["trays"]["assembly_thickness"] * MM
    body = static_body("assembly_tray", [x, y, ground_z + z])
    box(body, [aw, ad, thick], [0, 0, thick / 2], "assembly_tray", name="assembly_tray")
    ET.SubElement(body, "site", name="assembly_tray", pos=fmt([0, 0, thick]), group="4",
                  size="0.01")

    # Side table.
    tx, ty, top, sx, sy = cell.table
    tb = cell.config["table"]
    thick, leg, inset = tb["thickness"] * MM, tb["leg"] * MM, tb["leg_inset"] * MM
    body = static_body("table", [tx, ty, 0])
    box(body, [sx, sy, thick], [0, 0, ground_z + top - thick / 2], "table", name="table_top")
    leg_h = top - thick
    for ix in (-1, 1):
        for iy in (-1, 1):
            box(body, [leg, leg, leg_h],
                [ix * (sx / 2 - inset - leg / 2), iy * (sy / 2 - inset - leg / 2),
                 ground_z + leg_h / 2], "table_frame")

    # Carriers, each holding a stack.
    count = cell.config["stacks"]["sheets"]
    actuator = ET.SubElement(mujoco, "actuator")
    force = cell.config["carrier"]["clamp_force"]
    for carrier, stack, kind, x, y in cell.carriers:
        geo = cell.carrier(kind)
        height = geo["height"]
        frame_z = ground_z + top + geo["tine"] + 1e-4  # tine on the table
        body = static_body(carrier, [x, y, frame_z])
        ET.SubElement(body, "freejoint", name=carrier)
        for part, (size, centre, density) in geo["parts"].items():
            box(body, size, centre, "tine" if part == "tine" else "carrier",
                name=f"{carrier}_{part}", density=fmt(density), **STIFF,
                **(TINE if part == "tine" else {}))
        ET.SubElement(body, "site", name=f"{carrier}_handle", group="4", size="0.01",
                      pos=fmt(geo["handle"]))
        clamp = ET.SubElement(body, "body", name=f"{carrier}_clamp", pos=fmt([0, 0, height + 1e-4]))
        ET.SubElement(clamp, "joint", name=f"{carrier}_clamp", type="slide", axis="0 0 1",
                      range=fmt([CLAMP_CLOSED - 0.002, geo["open"] + 0.002]), armature="0.5",
                      damping="20")
        for part, (size, centre, density) in geo["clamp"].items():
            # The rod only shows the pad's guide: open, it rises into the
            # gripper's fingers, which a real rod would pass through a bore.
            extra = STIFF if part == "pad" else dict(contype="0", conaffinity="0", group="2")
            box(clamp, size, centre, "carrier", name=f"{carrier}_{part}", density=fmt(density),
                **extra)
        ET.SubElement(actuator, "position", name=f"{carrier}_clamp", joint=f"{carrier}_clamp",
                      kp="5000", kv="100", forcerange=fmt([-force, force]),
                      ctrlrange=fmt([CLAMP_CLOSED, geo["open"]]))

        body_xy, tab_xy, tab_centre, _ = cell.sheet(kind)
        s = cell.config["sheets"][kind]
        pkg = static_body(stack, [x, y, frame_z + height / 2 + 5e-5])
        ET.SubElement(pkg, "freejoint", name=stack)
        box(pkg, [*body_xy, height], [0, 0, 0], f"{kind}_coating", name=f"{stack}_body",
            density=fmt(s["density"]), **STIFF)
        box(pkg, [*tab_xy, height], [*tab_centre, 0], f"{kind}_tab", name=f"{stack}_tab",
            density=fmt(s["tab_density"]), **STIFF)
        pkg.append(ET.Comment(f" {count} {kind} sheets, {height * 1000:.1f} mm "))

    # Dock sites: where the base parks for each station (base_link height).
    d = cell.config["dock"]
    stations = [(n, cell.trays[n][0], cell.trays[n][1], d["tray_standoff"], d["tray_arm_offset"])
                for n in ("anode_tray", "assembly_tray", "cathode_tray")]
    stations += [(c, x, y, d["table_standoff"], d["table_arm_offset"])
                 for c, _, _, x, y in cell.carriers]
    for name, x, y, standoff, arm_offset in stations:
        bx, by, byaw = cell.dock(x, y, standoff, arm_offset)
        ET.SubElement(world, "site", name=f"{name}_dock", group="4", type="box",
                      size="0.08 0.01 0.005", **placement(np.array([bx, by, 0.0]),
                                                          rpy_matrix(0, 0, byaw)))

    ET.SubElement(world, "camera", name="overview",
                  **look_at(cell.to_world([1.2, 2.6, 2.2]), cell.to_world([-0.5, -0.35, 0.75])))
    # Close-ups of the carrier going in, from behind each tray (the robot
    # stands in front of it).
    for name in ("anode_tray", "cathode_tray"):
        x, y, z, _, _ = cell.trays[name]
        ET.SubElement(world, "camera", name=f"{name}_view", **look_at(
            cell.to_world([x + 0.35, y + 0.6, ground_z + z + 0.45]),
            cell.to_world([x + 0.08, y, ground_z + z + 0.03])))
    convert_urdf.add_keyframes(mujoco, keyframes)
    return mujoco


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--config", type=Path, default=CONFIG, help="cell layout")
    parser.add_argument("--task", type=Path, default=convert_urdf.DEFAULT_TASK,
                        help="task.yaml, for ground_z (the robot's base_link height)")
    parser.add_argument("--srdf", type=Path, default=convert_urdf.DEFAULT_SRDF,
                        help="named postures for the keyframes")
    parser.add_argument("--out", type=Path, default=OUT, help="scene file to write")
    args = parser.parse_args()

    robot = args.out.parent / "franzi.xml"
    if not robot.exists():
        raise SystemExit(f"{robot} is missing: run Mujoco/convert_urdf.py first")
    config = yaml.safe_load(args.config.read_text())
    cell = Cell(config)
    ground_z = yaml.safe_load(args.task.read_text())["pick_place_task"]["ros__parameters"]["ground_z"]
    meshdir = ET.parse(robot).getroot().find("compiler").get("meshdir")
    tray = (args.config.parent / config["trays"]["mesh"]).resolve()
    tray_mesh = os.path.relpath(tray, (args.out.parent / meshdir).resolve())

    import mujoco

    convert_urdf.write(build(cell, ground_z, {}, tray_mesh), args.out, "pem_cell.py")
    keyframes = convert_urdf.keyframe_vectors(args.out, convert_urdf.group_states(args.srdf))
    # Every keyframe starts with the carriers clamped shut.
    model = mujoco.MjModel.from_xml_path(str(args.out))
    for _, ctrl in keyframes.values():
        for carrier, *_ in cell.carriers:
            ctrl[model.actuator(f"{carrier}_clamp").id] = CLAMP_CLOSED
    convert_urdf.write(build(cell, ground_z, keyframes, tray_mesh), args.out, "pem_cell.py")

    model = mujoco.MjModel.from_xml_path(str(args.out))
    print(f"wrote  {args.out}")
    print(f"model: {model.nbody} bodies, {model.ngeom} geoms, {model.nu} actuators, "
          f"{model.nkey} keyframes; assembly tray {cell.trays['assembly_tray'][3] * 1000:.0f} x "
          f"{cell.trays['assembly_tray'][4] * 1000:.0f} mm")


if __name__ == "__main__":
    main()
