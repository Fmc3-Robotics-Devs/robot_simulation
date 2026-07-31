"""Abstract the Franzi (汇川双臂移动机器人) URDF into a primitive-geometry MJCF.

Every link becomes one box (wheels: cylinders) sized by its mesh's bounding
box, so MuJoCo never loads an STL. Kinematics, joint limits, masses and full
inertia tensors are carried over unchanged from the URDF; fixed sensor links
(cameras, lidars) are merged into their parent as visual-only geoms plus a
named site each. Output: franzi.xml next to this script.

Run:  python3 make_model.py
"""

import math
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
URDF = REPO / "Rviz/src/franzi_description/urdf/wheel_robot_4.0.urdf"
MESHES = REPO / "Rviz/src/franzi_description/meshes"
OUT = HERE / "franzi.xml"

SENSOR_HINTS = ("d435", "d405", "MID360", "2Dlidar")


def stl_bounds(path):
    """(min, max) corners of an STL, binary or ASCII, in the link frame."""
    data = path.read_bytes()
    if len(data) >= 84:
        (count,) = struct.unpack_from("<I", data, 80)
        if len(data) == 84 + 50 * count:  # binary
            lo = [float("inf")] * 3
            hi = [float("-inf")] * 3
            for i in range(count):
                base = 84 + 50 * i + 12  # skip the normal
                for v in range(3):
                    x, y, z = struct.unpack_from("<3f", data, base + 12 * v)
                    for k, val in enumerate((x, y, z)):
                        lo[k] = min(lo[k], val)
                        hi[k] = max(hi[k], val)
            return lo, hi
    verts = re.findall(
        rb"vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)", data
    )
    cols = list(zip(*[[float(v) for v in row] for row in verts]))
    return [min(c) for c in cols], [max(c) for c in cols]


def rpy_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def quat_rotate(q, v):
    w, x, y, z = q
    # v + 2*qvec x (qvec x v + w*v)
    tx = 2 * (y * v[2] - z * v[1])
    ty = 2 * (z * v[0] - x * v[2])
    tz = 2 * (x * v[1] - y * v[0])
    return (
        v[0] + w * tx + y * tz - z * ty,
        v[1] + w * ty + z * tx - x * tz,
        v[2] + w * tz + x * ty - y * tx,
    )


def fmt(values):
    return " ".join(f"{v:.6g}" for v in values)


def parse_urdf():
    root = ET.parse(URDF).getroot()
    links = {}
    for link in root.findall("link"):
        entry = {"mesh": None, "inertial": None, "rgba": "1 1 1 1"}
        visual = link.find("visual/geometry/mesh")
        if visual is not None:
            entry["mesh"] = MESHES / Path(visual.get("filename")).name
        colour = link.find("visual/material/color")
        if colour is not None:
            entry["rgba"] = colour.get("rgba")
        inertial = link.find("inertial")
        if inertial is not None:
            origin = inertial.find("origin")
            inertia = inertial.find("inertia")
            entry["inertial"] = {
                "pos": [float(v) for v in origin.get("xyz").split()],
                "mass": float(inertial.find("mass").get("value")),
                "fullinertia": [
                    float(inertia.get(k))
                    for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
                ],
            }
        links[link.get("name")] = entry

    joints = []
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")
        joints.append(
            {
                "name": joint.get("name"),
                "type": joint.get("type"),
                "parent": joint.find("parent").get("link"),
                "child": joint.find("child").get("link"),
                "pos": [float(v) for v in origin.get("xyz").split()],
                "rpy": [float(v) for v in origin.get("rpy").split()],
                "axis": [float(v) for v in axis.get("xyz").split()]
                if axis is not None
                else [0.0, 0.0, 1.0],
                "range": (
                    [float(limit.get("lower")), float(limit.get("upper"))]
                    if limit is not None and limit.get("lower") is not None
                    else None
                ),
                "effort": float(limit.get("effort")) if limit is not None else 100.0,
            }
        )
    return links, joints


LOGO_MODE = "--logo" in sys.argv

# Isaac parity: the exact paint scheme IssacSim/paint.py binds onto the
# robot (first matching regex wins) - gloss white panels, near-black visor,
# dark joint connectors and sensors, rubber-black tyres.
PAINT = [
    (r"head_pitch", "franzi_visor"),
    (r"finger|d435|d405|MID360|lidar", "franzi_joint"),
    (r"wheel_Link", "franzi_tire"),
    (r"waist|head_yaw|shoulder_pitch|elbow_pitch|wrist_roll|steering",
     "franzi_joint"),
    (r".", "franzi_white"),
]


def visual_look(name, links):
    """Isaac paint scheme by default; --logo switches to the branded
    palette (汇川 logo panels, orange lift column)."""
    if not LOGO_MODE:
        material = next(m for pattern, m in PAINT if re.search(pattern, name))
        return f'material="{material}"'
    if name in ("base_link", "torso_Link"):
        return 'material="franzi_logo"'
    if "calf" in name or "thigh" in name:
        return 'material="franzi_accent"'
    if any(k in name for k in
           ("shoulder_yaw", "wrist_yaw", "wrist_roll", "head_pitch")):
        return 'material="franzi_light"'
    return 'material="franzi_dark"'


def link_geom(name, links, indent):
    """Two geoms per link: the real STL as a contact-free visual (what Isaac
    renders), and its bounding box (wheels: cylinder) for collision."""
    mesh = links[name]["mesh"]
    if mesh is None or not mesh.exists():
        return []
    lo, hi = stl_bounds(mesh)
    centre = [(a + b) / 2 for a, b in zip(lo, hi)]
    ext = [max(b - a, 0.004) for a, b in zip(lo, hi)]
    pad = " " * indent
    wheel = "wheel" in name
    look = ('material="franzi_dark"'
            if wheel and LOGO_MODE else visual_look(name, links))
    out = [
        f'{pad}<geom name="{name}_visual" type="mesh" mesh="{name}" '
        f'{look} class="visual"/>'
    ]
    if wheel:  # cylinder along the link's y axis
        radius = max(ext[0], ext[2]) / 2
        out.append(
            f'{pad}<geom name="{name}" type="cylinder" size="{radius:.6g} '
            f'{ext[1] / 2:.6g}" pos="{fmt(centre)}" quat="0.707107 0.707107 0 0" '
            f'class="wheel"/>'
        )
    else:
        out.append(
            f'{pad}<geom name="{name}" type="box" size="{fmt(e / 2 for e in ext)}" '
            f'pos="{fmt(centre)}" class="collision"/>'
        )
    if name == "torso_Link" and not LOGO_MODE:
        # The FMC3 chest decal, placed like IssacSim/paint.spawn_logo: just
        # proud of the torso's front face, 60 % up its vertical extent.
        decal = (hi[0] + 0.002, (lo[1] + hi[1]) / 2,
                 lo[2] + 0.60 * (hi[2] - lo[2]))
        out.append(
            f'{pad}<geom name="chest_logo" type="box" size="0.0005 0.08 0.08" '
            f'pos="{fmt(decal)}" material="fmc3_logo" class="visual"/>'
        )
    return out


def main():
    links, joints = parse_urdf()
    children = {}
    for joint in joints:
        children.setdefault(joint["parent"], []).append(joint)

    lines = []

    def emit_inertial(name, indent):
        inertial = links[name]["inertial"]
        if inertial is None:
            return
        pad = " " * indent
        lines.append(
            f'{pad}<inertial pos="{fmt(inertial["pos"])}" '
            f'mass="{inertial["mass"]:.6g}" '
            f'fullinertia="{fmt(inertial["fullinertia"])}"/>'
        )

    actuators = []
    wheel_speed = 100.0 / 0.1  # effort-limited velocity servo on each wheel

    def emit_body(name, indent):
        pad = " " * indent
        emit_inertial(name, indent)
        lines.extend(link_geom(name, links, indent))
        for joint in children.get(name, ()):
            child = joint["child"]
            if joint["type"] == "fixed":
                # Cameras / lidars: a visual-only geom and a named site in the
                # parent body, no extra body.
                quat = rpy_to_quat(*joint["rpy"])
                mesh = links[child]["mesh"]
                site = child.replace("_Link", "")
                lines.append(
                    f'{pad}<site name="{site}" pos="{fmt(joint["pos"])}" '
                    f'quat="{fmt(quat)}" size="0.005"/>'
                )
                if mesh is not None and mesh.exists():
                    look = ('material="franzi_sensor"' if LOGO_MODE
                            else visual_look(child, links))
                    lines.append(
                        f'{pad}<geom name="{child}" type="mesh" mesh="{child}" '
                        f'pos="{fmt(joint["pos"])}" quat="{fmt(quat)}" '
                        f'{look} class="visual"/>'
                    )
                continue

            assert joint["rpy"] == [0.0, 0.0, 0.0], joint["name"]
            lines.append(f'{pad}<body name="{child}" pos="{fmt(joint["pos"])}">')
            jname = joint["name"]
            jtype = "slide" if joint["type"] == "prismatic" else "hinge"
            rng = (
                f' range="{fmt(joint["range"])}"'
                if joint["range"] is not None
                else ""
            )
            jclass = "wheel" if joint["type"] == "continuous" else "servo"
            lines.append(
                f'{pad}  <joint name="{jname}" type="{jtype}" '
                f'axis="{fmt(joint["axis"])}"{rng} class="{jclass}"/>'
            )
            if joint["type"] == "continuous":
                actuators.append(
                    f'    <velocity name="{jname}" joint="{jname}" kv="20" '
                    f'ctrlrange="-{wheel_speed:.6g} {wheel_speed:.6g}" '
                    f'forcerange="-{joint["effort"]:.6g} {joint["effort"]:.6g}"/>'
                )
            else:
                # The lift chain carries the ~75 kg upper body (gravity
                # gradient ~440 Nm/rad), so its servos must be much stiffer
                # than the gravity destabilisation or the stack buckles.
                if any(k in jname for k in ("calf", "thigh", "waist")):
                    kp, force = 6000, 800
                elif any(k in jname for k in ("shoulder", "elbow")):
                    kp, force = 1500, joint["effort"]
                elif "finger" in jname:
                    kp, force = 2000, joint["effort"]
                else:
                    kp, force = 400, joint["effort"]
                actuators.append(
                    f'    <position name="{jname}" joint="{jname}" kp="{kp}" '
                    f'ctrlrange="{fmt(joint["range"])}" '
                    f'forcerange="-{force:.6g} {force:.6g}"/>'
                )
            emit_body(child, indent + 2)
            lines.append(f"{pad}</body>")

    # Wheel contact height: front wheel centres sit at base_z + 0.1114 - 0.116.
    wheel_lo, wheel_hi = stl_bounds(MESHES / "left_front_wheel_Link.STL")
    wheel_radius = max(wheel_hi[0] - wheel_lo[0], wheel_hi[2] - wheel_lo[2]) / 2
    base_z = wheel_radius + 0.0046

    emit_body("base_link", 6)
    body_xml = "\n".join(lines)
    mesh_xml = "\n".join(
        f'    <mesh name="{name}" file="{entry["mesh"].name}"/>'
        for name, entry in links.items()
        if entry["mesh"] is not None and entry["mesh"].exists()
    )
    if LOGO_MODE:
        brand_xml = """\
    <!-- Cube texture: the logo panel repeats on every face. -->
    <texture name="franzi_logo" type="cube" file="logo.png"/>
    <material name="franzi_logo" texture="franzi_logo" specular="0.2"
              shininess="0.3"/>
    <material name="franzi_light" rgba="0.82 0.83 0.85 1" specular="0.3"
              shininess="0.4"/>
    <material name="franzi_dark" rgba="0.24 0.25 0.28 1" specular="0.4"
              shininess="0.5"/>
    <material name="franzi_accent" rgba="0.886 0.345 0.071 1" specular="0.3"
              shininess="0.4"/>
    <material name="franzi_sensor" rgba="0.16 0.22 0.4 1" specular="0.4"
              shininess="0.5"/>
"""
    else:
        # Isaac paint scheme (IssacSim/paint.py): rgb straight from its
        # palette, roughness mapped onto specular/shininess.
        brand_xml = """\
    <material name="franzi_white" rgba="0.85 0.86 0.88 1" specular="0.5"
              shininess="0.6"/>
    <material name="franzi_joint" rgba="0.09 0.095 0.105 1" specular="0.3"
              shininess="0.4"/>
    <material name="franzi_visor" rgba="0.02 0.02 0.025 1" specular="0.8"
              shininess="0.9"/>
    <material name="franzi_tire" rgba="0.04 0.04 0.045 1" specular="0.05"
              shininess="0.05"/>
    <texture name="fmc3_logo" type="cube" file="fmc3_logo.png"/>
    <material name="fmc3_logo" texture="fmc3_logo" specular="0.3"
              shininess="0.4"/>
"""
    actuator_xml = "\n".join(actuators)
    n_joints = sum(1 for j in joints if j["type"] != "fixed")
    home_qpos = fmt([0, 0, base_z, 1, 0, 0, 0] + [0.0] * n_joints)
    home_ctrl = fmt([0.0] * len(actuators))

    OUT.write_text(f"""<?xml version="1.0"?>
<!-- Generated by make_model.py from {URDF.relative_to(REPO)} - do not edit. -->
<mujoco model="franzi">
  <compiler angle="radian" balanceinertia="true" autolimits="true"
            texturedir="assets"
            meshdir="{os.path.relpath(MESHES, HERE)}"/>
  <option timestep="0.002" integrator="implicitfast"/>

  <asset>
{brand_xml}{mesh_xml}
  </asset>

  <default>
    <default class="franzi">
      <!-- Collision geoms collide with the world (contype 1) but never with
           each other (conaffinity 2): the boxes overlap at every joint.
           They live in group 3 (hidden by default); the real STL meshes are
           contact-free visuals in group 1.
           No rgba on classes with materials: rgba would override them. -->
      <geom contype="1" conaffinity="2" friction="0.9 0.005 0.0001"/>
      <joint damping="2" armature="0.1"/>
      <default class="visual">
        <geom contype="0" conaffinity="0" group="1"/>
      </default>
      <default class="collision">
        <geom group="3" rgba="0.7 0.9 0.4 0.4"/>
      </default>
      <default class="servo">
        <joint damping="5"/>
      </default>
      <default class="wheel">
        <joint damping="0.5" armature="0.02"/>
        <geom friction="1.2 0.01 0.0001" group="3" rgba="0.7 0.9 0.4 0.4"/>
      </default>
    </default>
  </default>

  <worldbody>
    <body name="base_link" pos="0 0 {base_z:.6g}" childclass="franzi">
      <freejoint name="base"/>
{body_xml}
    </body>
  </worldbody>

  <actuator>
{actuator_xml}
  </actuator>

  <keyframe>
    <key name="home" qpos="{home_qpos}" ctrl="{home_ctrl}"/>
  </keyframe>
</mujoco>
""")
    print(f"wrote {OUT} ({n_joints} joints, {len(actuators)} actuators, "
          f"wheel radius {wheel_radius * 1000:.1f} mm)")


if __name__ == "__main__":
    main()
