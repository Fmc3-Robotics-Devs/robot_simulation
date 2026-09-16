#!/usr/bin/env python3
"""Merge the two SolidWorks exports into the URDF the MuJoCo model is built from.

Neither export matches the robot on its own:

* ``wheel_robot_26.8.16_3/`` is the current export - chassis, legs, torso,
  head, shoulders and elbows are right. From the wrist-yaw joint on it is not:
  the gripper sits a quarter turn about the tool axis from the real one, the
  wrist D405 is somewhere else, and the wrist-yaw / wrist-pitch joints turn
  the other way.
* ``wheel_robot_7.24/`` (its URDF is still called ``wheel_robot_4.0``, the
  model the ROS stack used before) has the wrists and grippers right.

Both exports agree to within 0.005 mm on every mesh from the chassis up to
``wrist_pitch_Link`` at joint zero, and every arm joint axis is the same line
in both, so the merge is exact: take 26.8.16_3 and replace each
``{side}_wrist_yaw_Link`` subtree by the 7.24 one - its joints, frames,
limits, inertias and meshes verbatim. Only three things are new:

* **The graft.** ``{side}_wrist_yaw_joint`` moves from 7.24's
  ``elbow_pitch_Link`` frame into 26.8.16_3's (the two exports frame that link
  differently), found by forward kinematics at joint zero.
* **Names.** Links keep the current robot's names, which the MoveIt config
  uses: ``leftfinger1/2`` -> ``left_finger01/02``, ``left_wrist_d405`` ->
  ``left_D405`` (same on the right). Joint *signs* stay 7.24's.
* **Frames.** A REP-103 root (as in ``franzi_description``: the export stands
  the robot along +X, so its root becomes ``base_body_Link`` behind a massless
  ``base_link``), the SDK ``{side}_flange`` / ``{side}_tcp`` frames at the same
  physical place as in ``franzi_description``, and ``{side}_grasp``, the
  centre between the finger pads.

Writes ``franzi_merged/urdf/franzi_merged.urdf`` and copies the meshes it
uses to ``franzi_merged/meshes/`` (named after their links). Run from the
repository root with the Mujoco venv:

    Mujoco/.venv/bin/python Mujoco/merge_urdf.py
"""

import argparse
import copy
import math
import shutil
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from convert_urdf import fmt, origin_of, rpy_matrix  # noqa: E402

BODY_URDF = HERE / "wheel_robot_26.8.16_3" / "urdf" / "wheel_robot_26.8.16_3.urdf"
WRIST_URDF = HERE / "wheel_robot_7.24" / "urdf" / "wheel_robot_4.0.urdf"
NAME = "franzi_merged"
OUT = HERE / NAME
SIDES = ("left", "right")

# franzi_description's correction block: base_link -> base_body_Link.
ROOT_CORRECTION = ((0.0, 0.0, 0.24785), (0.0, -math.pi / 2, -math.pi / 2))
# 7.24 names -> current names, for links (+"_Link") and joints (+"_joint").
RENAME = {"{side}finger1": "{side}_finger01", "{side}finger2": "{side}_finger02",
          "{side}_wrist_d405": "{side}_D405"}
# SDK TCP: 273.5 mm along the flange's +x, rolled -90 deg (left) / +90 deg
# (right) about it. Hand-authored for the SDK, independent of the gripper.
SDK_TCP = {"left": ((0.2735, 0, 0), (-math.pi / 2, 0, 0)),
           "right": ((0.2735, 0, 0), (math.pi / 2, 0, 0))}


def transform(xyz, rot):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = rot, xyz
    return m


def rpy_of(rot):
    """URDF roll-pitch-yaw of a rotation matrix (inverse of rpy_matrix)."""
    rot = rot + 0.0  # no -0, which atan2 turns into -pi
    pitch = math.asin(max(-1.0, min(1.0, -rot[2, 0])))
    if abs(abs(rot[2, 0]) - 1) < 1e-9:  # gimbal lock: put it all in yaw
        return np.array([0.0, pitch, math.atan2(-rot[0, 1], rot[1, 1])])
    return np.array([math.atan2(rot[2, 1], rot[2, 2]), pitch, math.atan2(rot[1, 0], rot[0, 0])])


def snap(matrix, what, grid=1e-5, tolerance=(5e-5, 1e-4)):
    """Round a transform to its design value: a signed-permutation rotation
    and a 0.01 mm grid. The exports write pi/2 as 1.5708, which leaves
    micrometre / 10 microradian residue in composed transforms; anything
    larger is a real mismatch and stops the merge."""
    rot = np.round(matrix[:3, :3])
    xyz = np.round(matrix[:3, 3] / grid) * grid
    if not (np.allclose(rot @ rot.T, np.eye(3)) and np.linalg.det(rot) > 0):
        raise SystemExit(f"{what}: rotation is not a quarter-turn multiple:\n{matrix[:3, :3]}")
    error_p = np.abs(matrix[:3, 3] - xyz).max()
    error_r = np.abs(matrix[:3, :3] - rot).max()
    if error_p > tolerance[0] or error_r > tolerance[1]:
        raise SystemExit(f"{what}: {error_p:.1e} m / {error_r:.1e} off its design value")
    return xyz + 0.0, rot  # + 0.0: no -0


def origin_element(xyz, rot):
    return ET.Element("origin", xyz=fmt(xyz), rpy=fmt(rpy_of(rot), 12))


class Export:
    """One SolidWorks export: its XML and the tree over it."""

    def __init__(self, path):
        self.path = path
        self.root = ET.parse(path).getroot()
        self.links = {e.get("name"): e for e in self.root.findall("link")}
        self.joints = {e.find("child").get("link"): e for e in self.root.findall("joint")}
        self.children = {}
        for child, joint in self.joints.items():
            self.children.setdefault(joint.find("parent").get("link"), []).append(child)
        self.base = next(name for name in self.links if name not in self.joints)

    def subtree(self, link):
        names = [link]
        for child in self.children.get(link, []):
            names += self.subtree(child)
        return names

    def poses(self, base_pose):
        """Link poses at joint zero."""
        poses = {self.base: base_pose}

        def walk(link):
            for child in self.children.get(link, []):
                poses[child] = poses[link] @ transform(*origin_of(self.joints[child]))
                walk(child)
        walk(self.base)
        return poses

    def mesh(self, link):
        element = self.links[link].find("visual/geometry/mesh")
        if element is None:
            return None
        return self.path.parent.parent / "meshes" / Path(element.get("filename")).name


def renamed(name, side):
    for old, new in RENAME.items():
        old, new = old.format(side=side), new.format(side=side)
        for suffix in ("_Link", "_joint"):
            if name == old + suffix:
                return new + suffix
    return name


def frame(robot, name, parent, xyz, rot):
    ET.SubElement(robot, "link", name=name)
    joint = ET.SubElement(robot, "joint", name=f"{name}_joint", type="fixed")
    joint.append(origin_element(xyz, rot))
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=name)


def build(body_path=BODY_URDF, wrist_path=WRIST_URDF):
    """The merged URDF text and {mesh file name: source path}."""
    body, wrist = Export(body_path), Export(wrist_path)
    fix = transform(ROOT_CORRECTION[0], rpy_matrix(*ROOT_CORRECTION[1]))
    body_poses, wrist_poses = body.poses(fix), wrist.poses(np.eye(4))

    robot = ET.Element("robot", name=NAME)
    robot.append(ET.Comment(
        f" Generated by Mujoco/merge_urdf.py from {body_path.parent.parent.name} (chassis to elbows)"
        f" and {wrist_path.parent.parent.name} (wrists and grippers). Rerun it; do not edit. "))
    meshes = {}

    def copy_link(source, link, name):
        element = copy.deepcopy(source.links[link])
        element.set("name", name)
        mesh = source.mesh(link)
        if mesh is not None:
            meshes[f"{name}.STL"] = mesh
            for geometry in element.iter("mesh"):
                geometry.set("filename", f"package://{NAME}/meshes/{name}.STL")
        return element

    def copy_joint(source, child, rename=lambda name: name):
        element = copy.deepcopy(source.joints[child])
        element.set("name", rename(element.get("name")))
        for tag in ("parent", "child"):
            end = element.find(tag)
            end.set("link", rename(end.get("link")))
        return element

    # 1. REP-103 root in front of the export's chassis link.
    robot.append(ET.Comment(
        " REP-103 root, as in franzi_description: the export stands the robot along +X, so its"
        " root link is renamed base_body_Link and this massless base_link (x forward, z up, the"
        " origin task.yaml heights are measured from) is put in front of it. "))
    ET.SubElement(robot, "link", name="base_link")
    joint = ET.SubElement(robot, "joint", name="base_frame_joint", type="fixed")
    ET.SubElement(joint, "origin", xyz=fmt(ROOT_CORRECTION[0]), rpy=fmt(ROOT_CORRECTION[1], 12))
    ET.SubElement(joint, "parent", link="base_link")
    ET.SubElement(joint, "child", link="base_body_Link")

    # 2. The body export with each wrist subtree swapped, in document order.
    base = lambda name: "base_body_Link" if name == body.base else name  # noqa: E731
    replaced = {link: side for side in SIDES for link in body.subtree(f"{side}_wrist_yaw_Link")}
    for element in body.root:
        if element.tag not in ("link", "joint"):
            continue
        link = element.get("name") if element.tag == "link" else element.find("child").get("link")
        if link not in replaced:
            robot.append(copy_link(body, link, base(link)) if element.tag == "link"
                         else copy_joint(body, link, base))
            continue
        side = replaced[link]
        if element.tag == "joint" or link != f"{side}_wrist_yaw_Link":
            continue  # the rest of the 26.8.16_3 wrist: dropped
        robot.append(ET.Comment(
            f" {side} wrist and gripper: {wrist_path.parent.parent.name} verbatim (joint signs"
            " included); the fingers and the D405 take the current names "))
        elbow = f"{side}_elbow_pitch_Link"
        graft = (np.linalg.inv(body_poses[elbow]) @ wrist_poses[elbow]
                 @ transform(*origin_of(wrist.joints[link])))
        subtree = set(wrist.subtree(link))
        rename = lambda name, side=side: renamed(name, side)  # noqa: E731
        for part in wrist.root:
            if part.tag == "link" and part.get("name") in subtree:
                robot.append(copy_link(wrist, part.get("name"), rename(part.get("name"))))
            elif part.tag == "joint" and part.find("child").get("link") in subtree:
                joint = copy_joint(wrist, part.find("child").get("link"), rename)
                if joint.get("name") == f"{side}_wrist_yaw_joint":
                    # 7.24 frames elbow_pitch_Link differently: re-express
                    # the joint origin in 26.8.16_3's.
                    old = joint.find("origin")
                    joint.insert(list(joint).index(old),
                                 origin_element(*snap(graft, f"{side}_wrist_yaw_joint graft")))
                    joint.remove(old)
                robot.append(joint)

    # 3. Tool frames.
    robot.append(ET.Comment(
        " Tool frames. flange: the SDK flange, the J7 wrist-roll axis centre - the same physical"
        " frame as franzi_description's (there wrist_roll_Link itself). tcp: the SDK TCP, 273.5 mm"
        " along flange +x with opposite x rolls per side. grasp: centre between the finger pads at"
        " joint zero, +x the approach (along the tool), +z the direction finger01 closes in. "))
    for side in SIDES:
        roll = f"{side}_wrist_roll_Link"
        flange = snap(np.linalg.inv(wrist_poses[roll]) @ body_poses[roll], f"{side}_flange")
        frame(robot, f"{side}_flange", roll, *flange)
        xyz, rpy = SDK_TCP[side]
        frame(robot, f"{side}_tcp", f"{side}_flange", np.array(xyz, float), rpy_matrix(*rpy))

        # Grasp centre: middle of the two finger meshes' joint-zero bounding
        # box, in wrist_roll_Link.
        points = []
        for finger in (f"{side}finger1_Link", f"{side}finger2_Link"):
            pose = (np.linalg.inv(wrist_poses[roll]) @ wrist_poses[finger]
                    @ transform(*origin_of(wrist.links[finger].find("visual"))))
            points.append(read_stl(wrist.mesh(finger)) @ pose[:3, :3].T + pose[:3, 3])
        points = np.vstack(points)
        centre = np.round((points.min(0) + points.max(0)) / 2, 4) + 0.0
        finger = f"{side}finger1_Link"
        joint = wrist.joints[finger]
        axis = np.array([float(v) for v in joint.find("axis").get("xyz").split()])
        closes = 1.0 if float(joint.find("limit").get("upper")) > 0 else -1.0
        closing = (np.linalg.inv(wrist_poses[roll]) @ wrist_poses[finger])[:3, :3] @ axis * closes
        approach = flange[1][:, 0]
        rot = np.column_stack([approach, np.cross(closing, approach), closing])
        frame(robot, f"{side}_grasp", roll, centre,
              snap(transform(np.zeros(3), rot), f"{side}_grasp")[1])

    ET.indent(robot, "  ")
    text = '<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(robot, encoding="unicode") + "\n"
    return text, meshes


def read_stl(path):
    """Vertices (3n, 3) of a binary STL."""
    data = path.read_bytes()
    count = int.from_bytes(data[80:84], "little")
    record = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    return np.frombuffer(data[84:84 + count * 50], dtype=record)["vertices"].reshape(-1, 3).astype(float)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--body", type=Path, default=BODY_URDF, help="export for chassis to elbows")
    parser.add_argument("--wrist", type=Path, default=WRIST_URDF, help="export for wrists and grippers")
    parser.add_argument("--out", type=Path, default=OUT, help="output package directory")
    args = parser.parse_args()

    text, meshes = build(args.body, args.wrist)
    (args.out / "urdf").mkdir(parents=True, exist_ok=True)
    (args.out / "meshes").mkdir(exist_ok=True)
    urdf = args.out / "urdf" / f"{NAME}.urdf"
    urdf.write_text(text)
    for stale in (args.out / "meshes").glob("*.STL"):
        if stale.name not in meshes:
            stale.unlink()
    for name, source in meshes.items():
        shutil.copyfile(source, args.out / "meshes" / name)
    print(f"wrote {urdf} and {len(meshes)} meshes")


if __name__ == "__main__":
    main()
