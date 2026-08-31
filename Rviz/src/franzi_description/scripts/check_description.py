#!/usr/bin/env python3
"""Check the hand-applied corrections in the Franzi URDF still hold.

The URDF is a SolidWorks export plus a small block of corrections appended at
the end of the file (a REP-103 root, and the SDK flange/TCP frames). A
re-export overwrites everything above that block, so this script exists to
confirm the block was re-applied and still describes the same robot.

Run from the Rviz/ workspace root:

    python3 src/franzi_description/scripts/check_description.py
"""

import sys
from pathlib import Path
from xml.etree import ElementTree

import numpy as np


URDF = Path(__file__).resolve().parent.parent / "urdf" / "wheel_robot_26.8.16_3.urdf"


def rotation(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (
        np.cos(roll), np.sin(roll), np.cos(pitch),
        np.sin(pitch), np.cos(yaw), np.sin(yaw),
    )
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def transform(rot, translation):
    matrix = np.eye(4)
    matrix[:3, :3] = rot
    matrix[:3, 3] = translation
    return matrix


def forward_kinematics(root, joint_positions=()):
    """Link poses in the root frame, with the named joints displaced."""
    joint_positions = dict(joint_positions)
    joints = []
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        xyz = [float(v) for v in (origin.get("xyz") or "0 0 0").split()]
        rpy = [float(v) for v in (origin.get("rpy") or "0 0 0").split()]
        matrix = transform(rotation(*rpy), xyz)

        position = joint_positions.get(joint.get("name"))
        if position is not None:
            axis = np.array([float(v) for v in joint.find("axis").get("xyz").split()])
            if joint.get("type") == "prismatic":
                matrix = matrix @ transform(np.eye(3), axis * position)
            else:
                raise NotImplementedError("only prismatic joints are displaced here")
        joints.append(
            (joint.find("parent").get("link"), joint.find("child").get("link"), matrix)
        )

    children = {child for _, child, _ in joints}
    roots = [l.get("name") for l in root.findall("link") if l.get("name") not in children]
    if len(roots) != 1:
        raise AssertionError(f"expected exactly one root link, found {roots}")

    poses = {roots[0]: np.eye(4)}
    for _ in range(len(joints) + 1):
        for parent, child, matrix in joints:
            if parent in poses and child not in poses:
                poses[child] = poses[parent] @ matrix
    return roots[0], poses


def main():
    failures = []

    def check(condition, message):
        print(f"  {'ok  ' if condition else 'FAIL'}  {message}")
        if not condition:
            failures.append(message)

    root = ElementTree.parse(URDF).getroot()
    root_link, poses = forward_kinematics(root)
    links = {l.get("name") for l in root.findall("link")}

    print(f"{URDF.name}: {len(links)} links, {len(root.findall('joint'))} joints")

    print("\nstructure")
    check(root_link == "base_link", f"root link is base_link (got {root_link})")
    check(len(poses) == len(links), "every link is connected to the tree")
    missing = [
        m.get("filename")
        for m in root.iter("mesh")
        if not (URDF.parent.parent / m.get("filename").split("franzi_description/")[-1]).exists()
    ]
    check(not missing, f"every referenced mesh exists (missing: {missing})")

    # REP-103: x forward, y left, z up. The torso stacks straight up from the
    # chassis and the left arm is on +y; if the export's frame convention leaks
    # through, the robot lies along +x instead.
    print("\nbase_link is REP-103")
    torso = poses["torso_Link"][:3, 3]
    check(torso[2] > 0.9, f"torso is ~1 m above base_link in +z (z={torso[2]:.4f})")
    check(
        abs(torso[0]) < 1e-3 and abs(torso[1]) < 1e-3,
        f"torso is centred over base_link (x={torso[0]:.4f}, y={torso[1]:.4f})",
    )
    check(
        poses["left_wrist_roll_Link"][1, 3] > 0 > poses["right_wrist_roll_Link"][1, 3],
        "the left arm is on +y and the right arm on -y",
    )
    wheel = poses["left_front_wheel_Link"][:3, 3]
    check(wheel[0] > 0 and wheel[1] > 0, f"the left front wheel is at +x/+y ({wheel[:2].round(4)})")

    # The SDK flange is the J7 wrist-roll axis centre, which in this export is
    # the wrist_roll_Link frame itself; each TCP is 273.5 mm along flange +x.
    print("\nSDK flange / TCP frames")
    for side in ("left", "right"):
        if not {f"{side}_flange", f"{side}_tcp"} <= poses.keys():
            check(False, f"{side}_flange and {side}_tcp are defined")
            continue
        flange = np.linalg.inv(poses[f"{side}_wrist_roll_Link"]) @ poses[f"{side}_flange"]
        check(
            np.allclose(flange, np.eye(4), atol=1e-6),
            f"{side}_flange is identity on {side}_wrist_roll_Link",
        )
        tcp = np.linalg.inv(poses[f"{side}_flange"]) @ poses[f"{side}_tcp"]
        check(
            np.allclose(tcp[:3, 3], [0.2735, 0, 0], atol=1e-6),
            f"{side}_tcp is 273.5 mm along {side}_flange +x",
        )

    # Joint zero is fully open and the travel limits are fully closed, but the
    # sign that closes differs per side.
    print("\ngripper travel")
    for side, closed in (("left", (-0.0475, 0.0475)), ("right", (0.0475, -0.0475))):
        names = (f"{side}_finger01_joint", f"{side}_finger02_joint")
        gaps = []
        for values in ((0.0, 0.0), closed):
            _, pose = forward_kinematics(root, zip(names, values))
            gaps.append(
                np.linalg.norm(
                    pose[f"{side}_finger01_Link"][:3, 3] - pose[f"{side}_finger02_Link"][:3, 3]
                )
            )
        check(
            abs(gaps[0] - 0.095) < 1e-6 and gaps[1] < 1e-6,
            f"{side} gripper spans {gaps[0] * 1000:.1f} mm open and {gaps[1] * 1000:.1f} mm closed",
        )

    print()
    if failures:
        print(f"{len(failures)} check(s) failed")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
