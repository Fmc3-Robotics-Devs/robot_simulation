#!/usr/bin/env python3
"""Convert the Franzi URDF to MJCF for MuJoCo.

Writes two files next to each other:

* ``model/franzi.xml`` - the robot alone: bodies, meshes, joints, actuators,
  cameras and the named frames the ROS stack uses (flange/TCP, lidars).
* ``model/scene.xml``  - includes the robot and adds the floor, lights, a bench
  and a workpiece laid out from ``task.yaml``, plus the SRDF named postures as
  keyframes.

Either the raw SolidWorks export (``wheel_robot_26.8.16_3/``, the default) or
the corrected ``franzi_description`` URDF can be the input; both produce the
same model. What has to be got right, and why:

* **The export's root is not REP-103.** It stands the robot along +X. When the
  root link carries the chassis mesh it is renamed ``base_body_Link`` and a
  massless ``base_link`` is put in front of it with the same fixed transform
  ``franzi_description`` appends (see its README). ``base_link`` then keeps the
  origin ``task.yaml`` heights are measured against, so the MuJoCo world frame
  *is* the ``odom`` frame and the floor sits at ``ground_z``.
* **The base is holonomic, not wheeled.** Like the SRDF's planar
  ``world_joint``, ``base_link`` rides on x / y / yaw joints driven by
  integrated-velocity actuators. The steering and wheel joints are kept and
  driven to match the motion, but their geoms do not collide: three convex
  wheel hulls on a floor would fight the planar joints for no gain.
* **No self-collision.** Every link collides with the world as the convex hull
  of its mesh; hulls of neighbouring links overlap where the real meshes do
  not, so robot-robot pairs are filtered out. Self-collision is MoveIt's job.
* **Fixed links stay bodies.** The cameras and lidars hang off fixed joints;
  fusing them away would delete exactly the frames the sensors need.

Run from the repository root with the Mujoco venv:

    Mujoco/.venv/bin/python Mujoco/convert_urdf.py
"""

import argparse
import math
import os
import re
from pathlib import Path
from xml.dom import minidom
from xml.etree import ElementTree as ET

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
DEFAULT_URDF = HERE / "wheel_robot_26.8.16_3" / "urdf" / "wheel_robot_26.8.16_3.urdf"
DEFAULT_SRDF = (
    REPO / "Rviz" / "src" / "franzi_moveit_config" / "config" / "wheel_robot_26.8.16_3.srdf"
)
DEFAULT_TASK = REPO / "Rviz" / "src" / "franzi_pick_place" / "config" / "task.yaml"
DEFAULT_OUT = HERE / "model"

# franzi_description's correction block, applied when the input is the raw
# export (its root still carries the chassis mesh).
ROOT_CORRECTION = ((0.0, 0.0, 0.24785), (0.0, -math.pi / 2, -math.pi / 2))
# SDK flange = wrist_roll_Link itself; TCP 273.5 mm along flange +x, rolled
# -90 deg (left) / +90 deg (right) about x.
TOOL_FRAMES = {
    "left_flange": ("left_wrist_roll_Link", (0, 0, 0), (0, 0, 0)),
    "left_tcp": ("left_wrist_roll_Link", (0.2735, 0, 0), (-math.pi / 2, 0, 0)),
    "right_flange": ("right_wrist_roll_Link", (0, 0, 0), (0, 0, 0)),
    "right_tcp": ("right_wrist_roll_Link", (0.2735, 0, 0), (math.pi / 2, 0, 0)),
}

# Robot palette, after IssacSim/paint.py (the export paints every link white).
# First regex that matches the link name wins: (rgba, specular, shininess).
MATERIALS = {
    "white": ((0.85, 0.86, 0.88, 1), 0.5, 0.6),
    "dark": ((0.09, 0.095, 0.105, 1), 0.3, 0.3),
    "visor": ((0.02, 0.02, 0.025, 1), 0.9, 0.9),
    "tire": ((0.04, 0.04, 0.045, 1), 0.05, 0.05),
}
PALETTE = [
    (r"head_pitch", "visor"),
    (r"finger|d435|d405|MID360|lidar", "dark"),
    (r"wheel_Link", "tire"),
    (r"waist|head_yaw|shoulder_pitch|elbow_pitch|wrist_roll|steering|j0", "dark"),
    (r".", "white"),
]

# Cameras: (link, focal length, sensor width, resolution, optical roll). The
# lens numbers are IssacSim/ros2_cell.py's (RealSense D435 / D405). Every
# camera link points +z out of the lens, but its x axis is not the image row,
# so the optical frame is link * Rz(roll):
#   * D435s: Rz(180 deg). In this export the link's +x points to the robot's
#     left, so the half turn is what puts the robot's right on the image's
#     right and the floor at the bottom. (wheel_robot_4.0 needed Rz(-90 deg) -
#     the export re-rolled the camera links; check_model.py "cameras" tests
#     the images are upright.)
#   * D405s: Rz(+90 deg), which puts the gripper fingers at the bottom edge
#     of the picture, the usual wrist-camera view.
# A MuJoCo camera looks down its own -z with +y up: the extra x half-turn.
CAMERAS = {
    "head_d435": ("head_d435_Link", 1.93, 2.652, (1280, 720), math.pi),
    "body_d435": ("body_d435_Link", 1.93, 2.652, (640, 480), math.pi),
    "left_wrist_d405": ("left_D405_Link", 1.88, 2.782, (640, 480), math.pi / 2),
    "right_wrist_d405": ("right_D405_Link", 1.88, 2.782, (640, 480), math.pi / 2),
}

# Position-servo gains per joint group: (kp, kv, armature, damping). Sized
# from the joint-space inertia at "home" for roughly 4 Hz (body) to 15-20 Hz
# (arms) bandwidth, near critical damping. Every robot body has gravity
# compensation routed through its actuators (actuatorgravcomp), the way the
# real controller feeds gravity forward, so the servos only fight errors - and
# compensation plus servo together stay inside the URDF effort limit.
GAINS = [
    (r"calf|thigh|waist_pitch", (15000.0, 1500.0, 0.5, 20.0)),
    (r"waist_yaw", (3000.0, 150.0, 0.5, 20.0)),
    (r"shoulder|elbow", (1500.0, 60.0, 0.05, 2.0)),
    (r"wrist", (300.0, 10.0, 0.02, 1.0)),
    (r"head|steering", (100.0, 3.0, 0.01, 0.5)),
]
# Parallel gripper: one servo per hand on finger01, finger02 mirrored by an
# equality constraint. 100 N is the left export's finger effort; the right
# side exports 0 for effort and velocity, an exporter default.
GRIPPER = (2000.0, 40.0, 100.0)  # kp, kv, force limit
# Every joint in the export carries effort="100" velocity="3.14" - exporter
# placeholders, not actuator specs. For the arms 100 N m is ample (worst
# static load over the reach workspace ~25 N m), but the thigh needs ~110 N m
# just to hold the upper body up with an arm reaching forward, so the body
# lift joints get an assumed 400 N m. Replace with the real spec when known.
BODY_EFFORT = (r"calf|thigh|waist_pitch", 400.0)


def rpy_matrix(roll, pitch, yaw):
    """URDF fixed-axis roll-pitch-yaw: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def quat_from_matrix(m):
    """Unit quaternion (w, x, y, z) of a rotation matrix, w >= 0."""
    trace = np.trace(m)
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
        q = [0.0] * 4
        q[0] = (m[k, j] - m[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (m[j, i] + m[i, j]) / s
        q[1 + k] = (m[k, i] + m[i, k]) / s
    q = np.array(q)
    q /= np.linalg.norm(q)
    return q if q[0] >= 0 else -q


def fmt(values, digits=8):
    """Space-separated numbers, rounded, without '-0'."""
    out = []
    for value in np.atleast_1d(values):
        text = f"{float(value):.{digits}g}"
        out.append("0" if text in ("-0", "0") else text)
    return " ".join(out)


def origin_of(element):
    """(xyz, rotation matrix) of a URDF <origin>, identity if absent."""
    origin = element.find("origin") if element is not None else None
    if origin is None:
        return np.zeros(3), np.eye(3)
    xyz = np.array([float(v) for v in (origin.get("xyz") or "0 0 0").split()])
    rpy = [float(v) for v in (origin.get("rpy") or "0 0 0").split()]
    return xyz, rpy_matrix(*rpy)


def placement(xyz, rot):
    """MJCF pos/quat attributes, omitting identities."""
    attrs = {}
    if np.linalg.norm(xyz) > 1e-12:
        attrs["pos"] = fmt(xyz)
    quat = quat_from_matrix(rot)
    if not np.allclose(quat, [1, 0, 0, 0], atol=1e-12):
        attrs["quat"] = fmt(quat)
    return attrs


def style(link):
    return next(name for pattern, name in PALETTE if re.search(pattern, link, re.IGNORECASE))


def gains(joint):
    return next((g for pattern, g in GAINS if re.search(pattern, joint)), None)


class Urdf:
    """The parts of a URDF this converter needs, as a tree."""

    def __init__(self, path):
        self.path = Path(path)
        root = ET.parse(self.path).getroot()
        self.links = {link.get("name"): link for link in root.findall("link")}
        self.joints = {}
        self.children = {}
        for joint in root.findall("joint"):
            parent = joint.find("parent").get("link")
            self.joints[joint.find("child").get("link")] = joint
            self.children.setdefault(parent, []).append(joint.find("child").get("link"))
        roots = [name for name in self.links if name not in self.joints]
        if len(roots) != 1:
            raise SystemExit(f"expected one root link, found {roots}")
        self.root = roots[0]

    def mesh_file(self, link, kind):
        element = self.links[link].find(f"{kind}/geometry/mesh")
        if element is None:
            return None
        # package://<pkg>/meshes/x.STL -> <urdf dir>/../meshes/x.STL
        name = Path(element.get("filename")).name
        return (self.path.parent.parent / "meshes" / name).resolve()

    def is_frame(self, link):
        """A massless, geometry-free link: a named frame, not a body."""
        element = self.links[link]
        return all(element.find(tag) is None for tag in ("inertial", "visual", "collision"))


def build_robot(urdf, meshdir_rel):
    mujoco = ET.Element("mujoco", model="franzi")
    ET.SubElement(
        mujoco,
        "compiler",
        angle="radian",
        meshdir=meshdir_rel,
        autolimits="true",
        # The export's inertias come straight from CAD: a few are degenerate
        # (2Dlidar_Link is all zero, head_pitch_Link fails the triangle
        # inequality). Bound and balance them rather than refuse the model.
        balanceinertia="true",
        boundmass="1e-4",
        boundinertia="1e-7",
    )

    default = ET.SubElement(ET.SubElement(mujoco, "default"), "default", {"class": "franzi"})
    ET.SubElement(default, "joint", damping="1", armature="0.01", actuatorgravcomp="true")
    ET.SubElement(
        ET.SubElement(default, "default", {"class": "visual"}),
        "geom",
        type="mesh", contype="0", conaffinity="0", group="2", density="0",
    )
    # contype 1 / conaffinity 0: collides with the world (which is 1/1) but
    # never with another robot geom.
    collision = ET.SubElement(default, "default", {"class": "collision"})
    ET.SubElement(collision, "geom", type="mesh", contype="1", conaffinity="0", group="3",
                  density="0", rgba="0.9 0.4 0.2 0.4")
    ET.SubElement(
        ET.SubElement(collision, "default", {"class": "finger"}),
        # Rubber pads: high friction, torsional friction so a held part does
        # not spin about the grasp axis, and slightly soft contacts.
        "geom", friction="1.5 0.02 0.001", condim="4", solref="0.004 1", priority="1",
    )
    ET.SubElement(ET.SubElement(default, "default", {"class": "wheel"}), "geom",
                  type="mesh", contype="0", conaffinity="0", group="3", density="0",
                  rgba="0.9 0.4 0.2 0.4")

    asset = ET.SubElement(mujoco, "asset")
    for name, (rgba, specular, shininess) in MATERIALS.items():
        ET.SubElement(asset, "material", name=f"franzi_{name}", rgba=fmt(rgba, 4),
                      specular=fmt(specular), shininess=fmt(shininess))

    meshes = {}
    worldbody = ET.SubElement(mujoco, "worldbody")
    sites_needed = dict(TOOL_FRAMES)

    def add_geoms(body, link):
        # Visual and collision reference the same STL in this export.
        mesh = urdf.mesh_file(link, "visual")
        if mesh is None:
            return
        if not mesh.exists():
            raise SystemExit(f"{link}: mesh not found: {mesh}")
        name = mesh.stem if mesh.stem != "base_link" else "base_body_Link"
        if name not in meshes:
            meshes[name] = mesh.name
            ET.SubElement(asset, "mesh", name=name, file=mesh.name)
        xyz, rot = origin_of(urdf.links[link].find("visual"))
        where = placement(xyz, rot)
        ET.SubElement(body, "geom", {"class": "visual", "mesh": name,
                                     "material": f"franzi_{style(link)}", **where})
        is_wheel = re.search(r"wheel_Link|steering_Link", link)
        cls = "wheel" if is_wheel else "finger" if "finger" in link else "collision"
        xyz, rot = origin_of(urdf.links[link].find("collision"))
        ET.SubElement(body, "geom", {"class": cls, "mesh": name, "name": f"{link}_collision",
                                     **placement(xyz, rot)})

    def add_inertial(body, link):
        inertial = urdf.links[link].find("inertial")
        if inertial is None:
            return
        xyz, rot = origin_of(inertial)
        i = inertial.find("inertia")
        get = lambda key: float(i.get(key))  # noqa: E731
        tensor = np.array([
            [get("ixx"), get("ixy"), get("ixz")],
            [get("ixy"), get("iyy"), get("iyz")],
            [get("ixz"), get("iyz"), get("izz")],
        ])
        tensor = rot @ tensor @ rot.T  # into the link frame
        mass = float(inertial.find("mass").get("value"))
        if np.linalg.eigvalsh(tensor).min() <= 0:
            # 2Dlidar_Link exports an all-zero tensor. Stand in a 3 cm solid
            # sphere of the same mass; it is welded to the chassis anyway.
            print(f"  {link}: degenerate inertia {np.diag(tensor)}, using a 3 cm sphere")
            ET.SubElement(body, "inertial", pos=fmt(xyz), mass=fmt(mass),
                          diaginertia=fmt([0.4 * mass * 0.03**2] * 3))
            return
        ET.SubElement(
            body, "inertial",
            pos=fmt(xyz),
            mass=fmt(mass),
            fullinertia=fmt([tensor[0, 0], tensor[1, 1], tensor[2, 2],
                             tensor[0, 1], tensor[0, 2], tensor[1, 2]]),
        )

    def add_joint(body, link):
        joint = urdf.joints[link]
        kind = joint.get("type")
        if kind == "fixed":
            return None
        axis = joint.find("axis")
        attrs = {
            "name": joint.get("name"),
            "type": "slide" if kind == "prismatic" else "hinge",
            "axis": fmt([float(v) for v in axis.get("xyz").split()]) if axis is not None else "0 0 1",
        }
        limit = joint.find("limit")
        if kind != "continuous" and limit is not None:
            attrs["range"] = fmt([float(limit.get("lower")), float(limit.get("upper"))])
        g = gains(joint.get("name"))
        if g is not None:
            attrs["armature"], attrs["damping"] = fmt(g[2]), fmt(g[3])
        if limit is not None and float(limit.get("effort") or 0) > 0:
            effort = float(limit.get("effort"))
            if re.search(BODY_EFFORT[0], joint.get("name")):
                effort = max(effort, BODY_EFFORT[1])
            attrs["actuatorfrcrange"] = fmt([-effort, effort])
        if "finger" in joint.get("name"):
            attrs["armature"], attrs["damping"] = "0.001", "5"
            attrs["actuatorfrcrange"] = fmt([-GRIPPER[2], GRIPPER[2]])
        ET.SubElement(body, "joint", attrs)
        return joint.get("name")

    joints = []

    def add_frame(parent, link, xyz, rot):
        """A frame link becomes a site; frames hanging off it compose onto it."""
        ET.SubElement(parent, "site", {"name": link, "group": "4", "size": "0.01",
                                       **placement(xyz, rot)})
        sites_needed.pop(link, None)
        for child in urdf.children.get(link, []):
            if not urdf.is_frame(child):
                raise SystemExit(f"{child} hangs off the frame {link} but has a body")
            xyz_c, rot_c = origin_of(urdf.joints[child])
            add_frame(parent, child, xyz + rot @ xyz_c, rot @ rot_c)

    def add_link(parent, link, xyz, rot):
        if urdf.is_frame(link):
            add_frame(parent, link, xyz, rot)
            return
        body = ET.SubElement(parent, "body", {"name": link, "gravcomp": "1",
                                              **placement(xyz, rot)})
        if link != "base_link":
            name = add_joint(body, link)
            if name:
                joints.append(name)
        add_inertial(body, link)
        add_geoms(body, link)
        for child in urdf.children.get(link, []):
            xyz_c, rot_c = origin_of(urdf.joints[child])
            add_link(body, child, xyz_c, rot_c)
        return body

    # base_link: the holonomic base rides on the world with three joints.
    base = ET.SubElement(worldbody, "body", {"name": "base_link", "childclass": "franzi"})
    for name, kind, axis in (("base_x", "slide", "1 0 0"), ("base_y", "slide", "0 1 0"),
                             ("base_yaw", "hinge", "0 0 1")):
        ET.SubElement(base, "joint", name=name, type=kind, axis=axis, damping="0",
                      armature="0", limited="false")
    ET.SubElement(base, "site", name="base_link", group="4", size="0.02")
    ET.SubElement(base, "camera", name="chase", mode="trackcom", pos="-2.6 -1.6 1.6",
                  xyaxes="0.52 -0.85 0 0.3 0.18 0.94")

    if urdf.is_frame(urdf.root):  # franzi_description: already corrected
        for child in urdf.children.get(urdf.root, []):
            xyz, rot = origin_of(urdf.joints[child])
            if child != "base_body_Link":
                raise SystemExit(f"unexpected child of base_link: {child}")
            add_link(base, child, xyz, rot)
    else:  # raw export: its root is the chassis
        urdf.links["base_body_Link"] = urdf.links.pop(urdf.root)
        urdf.children["base_body_Link"] = urdf.children.pop(urdf.root)
        urdf.root = "base_body_Link"
        xyz, rpy = ROOT_CORRECTION
        body = ET.SubElement(base, "body", {"name": "base_body_Link", "gravcomp": "1",
                                            **placement(np.array(xyz), rpy_matrix(*rpy))})
        add_inertial(body, "base_body_Link")
        add_geoms(body, "base_body_Link")
        for child in urdf.children["base_body_Link"]:
            xyz_c, rot_c = origin_of(urdf.joints[child])
            add_link(body, child, xyz_c, rot_c)

    # Tool frames the raw export does not have.
    for name, (parent, xyz, rpy) in sites_needed.items():
        body = worldbody.find(f".//body[@name='{parent}']")
        ET.SubElement(body, "site", {"name": name, "group": "4", "size": "0.01",
                                     **placement(np.array(xyz, float), rpy_matrix(*rpy))})

    # Cameras.
    for name, (link, focal, width, (px, py), roll) in CAMERAS.items():
        view = rpy_matrix(0, 0, roll) @ rpy_matrix(math.pi, 0, 0)
        body = worldbody.find(f".//body[@name='{link}']")
        height = width * py / px
        ET.SubElement(body, "camera", {
            "name": name, **placement(np.zeros(3), view),
            "focal": fmt([focal * 1e-3] * 2), "sensorsize": fmt([width * 1e-3, height * 1e-3]),
            "resolution": f"{px} {py}",
        })
    for link in ("2Dlidar_Link", "MID360_Link"):
        body = worldbody.find(f".//body[@name='{link}']")
        ET.SubElement(body, "site", name=link.replace("_Link", ""), group="4", size="0.01")

    # Finger coupling and actuators.
    equality = ET.SubElement(mujoco, "equality")
    actuator = ET.SubElement(mujoco, "actuator")
    # Base: integrated velocity -> the pose holds when the command is zero.
    # ~150 kg robot: 20 kN/m and 2 kN s/m settle a step in ~0.3 s; the force
    # limits allow ~1 g of acceleration, far above what the base is asked for.
    for name, kp, kv, force in (("base_x", 20000.0, 2000.0, 1500.0),
                                ("base_y", 20000.0, 2000.0, 1500.0),
                                ("base_yaw", 5000.0, 500.0, 500.0)):
        ET.SubElement(actuator, "intvelocity", name=name, joint=name, kp=fmt(kp),
                      kv=fmt(kv), actrange="-1000 1000", ctrlrange="-2 2",
                      forcerange=fmt([-force, force]))
    for joint_name in joints:
        joint = next(j for j in urdf.joints.values() if j.get("name") == joint_name)
        limit = joint.find("limit")
        if "finger01" in joint_name:
            side = joint_name.split("_")[0]
            ET.SubElement(equality, "joint", joint1=f"{side}_finger02_joint", joint2=joint_name,
                          polycoef="0 -1 0 0 0", solref="0.005 1")
            kp, kv, _ = GRIPPER
            ET.SubElement(actuator, "position", name=f"{side}_gripper", joint=joint_name,
                          kp=fmt(kp), kv=fmt(kv),
                          ctrlrange=fmt([float(limit.get("lower")), float(limit.get("upper"))]))
            continue
        if "finger" in joint_name:
            continue
        if joint.get("type") == "continuous":  # wheels: velocity servo
            ET.SubElement(actuator, "velocity", name=joint_name, joint=joint_name, kv="2",
                          ctrlrange="-100 100", forcerange="-50 50")
            continue
        kp, kv, _, _ = gains(joint_name)
        ET.SubElement(actuator, "position", name=joint_name, joint=joint_name,
                      kp=fmt(kp), kv=fmt(kv),
                      ctrlrange=fmt([float(limit.get("lower")), float(limit.get("upper"))]))

    sensor = ET.SubElement(mujoco, "sensor")
    for side in ("left", "right"):
        for kind in ("framepos", "framequat"):
            ET.SubElement(sensor, kind, name=f"{side}_tcp_{kind[5:]}", objtype="site",
                          objname=f"{side}_tcp")
    return mujoco


def look_at(eye, target):
    """MJCF camera pos/xyaxes looking from ``eye`` at ``target``, z up."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    forward = (target - eye) / np.linalg.norm(target - eye)
    right = np.cross(forward, [0, 0, 1])
    right /= np.linalg.norm(right)
    return {"pos": fmt(eye), "xyaxes": fmt(np.concatenate([right, np.cross(right, forward)]), 4)}


def group_states(srdf):
    """{(group, state): {joint: value}} from the SRDF."""
    states = {}
    for state in ET.parse(srdf).getroot().findall("group_state"):
        states[(state.get("group"), state.get("name"))] = {
            j.get("name"): float(j.get("value")) for j in state.findall("joint")
        }
    return states


SCENE_EXTENT = 2.2  # m; MuJoCo scales clip planes and the free camera by it

# Keyframes: SRDF named states combined into whole-robot postures.
KEYFRAMES = {
    "home": [("body", "work"), ("head", "home"), ("both_arms", "off_arms"),
             ("left_gripper", "open"), ("right_gripper", "open")],
    "look_down": [("body", "work"), ("head", "look_down"), ("both_arms", "off_arms"),
                  ("left_gripper", "open"), ("right_gripper", "open")],
    "zero": [("body", "off_body"), ("head", "home"), ("both_arms", "off_arms"),
             ("left_gripper", "open"), ("right_gripper", "open")],
}


def build_scene(task, keyframes):
    p = task["pick_place_task"]["ros__parameters"]
    ground_z = p["ground_z"]
    bench, part = p["bench"], p["workpiece"]
    dock_x, dock_y = p["dock"]["offset"]

    mujoco = ET.Element("mujoco", model="franzi_scene")
    ET.SubElement(mujoco, "include", file="franzi.xml")
    ET.SubElement(mujoco, "option", timestep="0.002", integrator="implicitfast",
                  cone="elliptic", impratio="10")
    ET.SubElement(mujoco, "statistic", center="0.3 0 0.5", extent=fmt(SCENE_EXTENT))
    visual = ET.SubElement(mujoco, "visual")
    ET.SubElement(visual, "headlight", diffuse="0.5 0.5 0.5", ambient="0.35 0.35 0.35",
                  specular="0 0 0")
    ET.SubElement(visual, "global", azimuth="150", elevation="-20", offwidth="1280",
                  offheight="720")
    ET.SubElement(visual, "quality", shadowsize="4096")
    # 5 cm near clip, the D435's minimum range (and Isaac's clipping_range).
    # The head camera sits behind the visor shell, 2-3 cm in front of the lens
    # where the real robot has a window; a closer plane renders its inside.
    ET.SubElement(visual, "map", znear=fmt(0.05 / SCENE_EXTENT, 4))

    asset = ET.SubElement(mujoco, "asset")
    ET.SubElement(asset, "texture", type="skybox", builtin="gradient", rgb1="0.35 0.4 0.45",
                  rgb2="0.05 0.06 0.08", width="512", height="512")
    ET.SubElement(asset, "texture", name="floor", type="2d", builtin="checker",
                  rgb1="0.32 0.33 0.35", rgb2="0.26 0.27 0.29", width="512", height="512",
                  mark="edge", markrgb="0.4 0.4 0.42")
    ET.SubElement(asset, "material", name="floor", texture="floor", texrepeat="8 8",
                  reflectance="0.05")
    ET.SubElement(asset, "material", name="bench", rgba="0.25 0.27 0.3 1")
    ET.SubElement(asset, "material", name="aluminium", rgba="0.75 0.77 0.8 1",
                  specular="0.8", shininess="0.8")

    world = ET.SubElement(mujoco, "worldbody")
    ET.SubElement(world, "light", pos="0 0 4", dir="0 0 -1", diffuse="0.6 0.6 0.6",
                  castshadow="true")
    ET.SubElement(world, "light", pos="3 -3 3", dir="-1 1 -1", diffuse="0.3 0.3 0.3",
                  castshadow="false")
    # The floor is where the wheels touch: task.yaml's ground_z in odom.
    ET.SubElement(world, "geom", name="floor", type="plane", size="0 0 0.05",
                  pos=fmt([0, 0, ground_z]), material="floor")

    # One bench at the dock offset, near edge part_inset in front of the part:
    # the posture the robot parks in at every station.
    sx, sy = bench["size_xy"]
    top, thick = bench["top_z"], bench["thickness"]
    near_x = dock_x - bench["part_inset"] - part["size"][0] / 2
    cx = near_x + sx / 2
    body = ET.SubElement(world, "body", name="bench", pos=fmt([cx, dock_y, 0]))
    ET.SubElement(body, "geom", name="bench_top", type="box", material="bench",
                  size=fmt([sx / 2, sy / 2, thick / 2]), pos=fmt([0, 0, top - thick / 2]))
    leg = bench["leg_size"]
    leg_h = top - thick - ground_z
    for ix in (-1, 1):
        for iy in (-1, 1):
            ET.SubElement(body, "geom", type="box", material="bench",
                          size=fmt([leg / 2, leg / 2, leg_h / 2]),
                          pos=fmt([ix * (sx / 2 - bench["leg_inset"] - leg / 2),
                                   iy * (sy / 2 - bench["leg_inset"] - leg / 2),
                                   ground_z + leg_h / 2]))

    size = np.array(part["size"])
    workpiece = ET.SubElement(world, "body", name="workpiece",
                              pos=fmt([dock_x, dock_y, top + size[2] / 2 + 1e-4]))
    ET.SubElement(workpiece, "freejoint", name="workpiece")
    ET.SubElement(workpiece, "geom", name="workpiece", type="box", size=fmt(size / 2),
                  material="aluminium", density="2700", friction="1 0.02 0.001", condim="4")

    # Observer camera across the aisle, framing the start pose and the bench.
    ET.SubElement(world, "camera", name="overview", **look_at((2.3, -2.9, 2.1), (-0.25, 0.0, 0.6)))

    keyframe = ET.SubElement(mujoco, "keyframe")
    for name, (qpos, ctrl) in keyframes.items():
        ET.SubElement(keyframe, "key", name=name, qpos=fmt(qpos, 6), ctrl=fmt(ctrl, 6))
    return mujoco


def keyframe_vectors(scene_path, states):
    """Full qpos/ctrl vectors for each keyframe, by compiling the scene once."""
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    out = {}
    for name, parts in KEYFRAMES.items():
        mujoco.mj_resetData(model, data)
        values = {}
        for key in parts:
            values.update(states[key])
        for joint, value in values.items():
            data.qpos[model.jnt_qposadr[model.joint(joint).id]] = value
        for joint in values:
            if "finger02" in joint:
                continue
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                    joint.replace("finger01_joint", "gripper"))
            if aid >= 0:
                data.ctrl[aid] = values[joint]
        out[name] = (data.qpos.copy(), data.ctrl.copy())
    return out


def write(element, path):
    text = minidom.parseString(ET.tostring(element)).toprettyxml(indent="  ")
    header = "<!-- Generated by Mujoco/convert_urdf.py - edit the converter, not this file. -->\n"
    path.write_text(header + "\n".join(text.splitlines()[1:]) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF,
                        help="raw export or franzi_description URDF")
    parser.add_argument("--srdf", type=Path, default=DEFAULT_SRDF, help="named postures")
    parser.add_argument("--task", type=Path, default=DEFAULT_TASK, help="cell layout")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help="output directory")
    args = parser.parse_args()

    urdf = Urdf(args.urdf)
    args.out.mkdir(parents=True, exist_ok=True)
    # Relative, so the model directory can move with the repository.
    meshdir = os.path.relpath(args.urdf.resolve().parent.parent / "meshes", args.out.resolve())
    robot = build_robot(urdf, meshdir)
    write(robot, args.out / "franzi.xml")

    task = yaml.safe_load(args.task.read_text())
    states = group_states(args.srdf)
    scene_path = args.out / "scene.xml"
    write(build_scene(task, {}), scene_path)
    write(build_scene(task, keyframe_vectors(scene_path, states)), scene_path)

    import mujoco

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    print(f"input  {args.urdf}")
    print(f"wrote  {args.out / 'franzi.xml'}")
    print(f"wrote  {scene_path}")
    print(f"model: {model.nbody} bodies, {model.njnt} joints, {model.nu} actuators, "
          f"{model.nmesh} meshes, {model.ncam} cameras, {model.nkey} keyframes")


if __name__ == "__main__":
    main()
