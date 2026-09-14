#!/usr/bin/env python3
"""Check the MuJoCo model against the rest of the repository and physics.

Each section tests one claim the README makes, against an independent
reference where there is one:

* conversion  - the raw export and franzi_description produce the same MJCF
* kinematics  - MuJoCo FK equals the franzi_description URDF's, link by link
* ground      - wheels touch the floor at task.yaml's ground_z, radius matches
* gravity     - every keyframe holds still within the joint effort limits
* gripper     - 95 mm open, 0 mm closed, both sides
* base        - drive_to lands, the swerve modules steer along the motion
* cameras     - the D435 images are upright, the wrist cameras see the fingers
* scan        - the 2D lidar sees the bench legs where they are
* pick        - demo.py's pick-and-place actually moves the part by physics

Run from the repository root (needs no ROS; rendering uses EGL when there is
no display):

    MUJOCO_GL=egl Mujoco/.venv/bin/python Mujoco/check_model.py
"""

import contextlib
import io
import math
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import convert_urdf  # noqa: E402
import demo  # noqa: E402
from franzi import Franzi  # noqa: E402

REFERENCE_URDF = (HERE.parent / "Rviz" / "src" / "franzi_description" / "urdf"
                  / "wheel_robot_26.8.16_3.urdf")

failures = []


def check(condition, message):
    print(f"  {'ok  ' if condition else 'FAIL'}  {message}")
    if not condition:
        failures.append(message)


def urdf_fk(path, positions):
    """Link poses in base_link from the URDF, with joints at ``positions``."""
    root = ET.parse(path).getroot()
    joints = []
    for joint in root.findall("joint"):
        xyz, rot = convert_urdf.origin_of(joint)
        matrix = np.eye(4)
        matrix[:3, :3], matrix[:3, 3] = rot, xyz
        q = positions.get(joint.get("name"), 0.0)
        axis = joint.find("axis")
        if joint.get("type") != "fixed" and q:
            a = np.array([float(v) for v in axis.get("xyz").split()])
            motion = np.eye(4)
            if joint.get("type") == "prismatic":
                motion[:3, 3] = a * q
            else:  # Rodrigues
                k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                motion[:3, :3] = np.eye(3) + math.sin(q) * k + (1 - math.cos(q)) * k @ k
            matrix = matrix @ motion
        joints.append((joint.find("parent").get("link"), joint.find("child").get("link"), matrix))
    poses = {"base_link": np.eye(4)}
    for _ in range(len(joints)):
        for parent, child, matrix in joints:
            if parent in poses and child not in poses:
                poses[child] = poses[parent] @ matrix
    return poses


def conversion():
    print("\nconversion: raw export and franzi_description agree")
    with contextlib.redirect_stdout(io.StringIO()):  # the converter's notes
        texts = [ET.tostring(convert_urdf.build_robot(convert_urdf.Urdf(path), "meshes"))
                 for path in (convert_urdf.DEFAULT_URDF, REFERENCE_URDF)]
    check(texts[0] == texts[1], "both inputs produce byte-identical franzi.xml")


def kinematics(robot):
    print("\nkinematics: MuJoCo FK = franzi_description URDF FK")
    m, d = robot.model, robot.data
    hinge = [j for j in range(m.njnt) if m.jnt_type[j] in (2, 3)
             and not m.joint(j).name.startswith("base_")]
    rng = np.random.default_rng(1)
    worst_p = worst_r = 0.0
    for _ in range(10):
        positions = {}
        for j in hinge:
            low, high = m.jnt_range[j] if m.jnt_limited[j] else (-math.pi, math.pi)
            positions[m.joint(j).name] = rng.uniform(low, high)
            d.qpos[m.jnt_qposadr[j]] = positions[m.joint(j).name]
        d.qpos[:3] = 0
        mujoco.mj_kinematics(m, d)
        poses = urdf_fk(REFERENCE_URDF, positions)
        for name, pose in poses.items():
            if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name) >= 0:
                pos, rot = d.body(name).xpos, d.body(name).xmat.reshape(3, 3)
            else:
                pos, rot = d.site(name).xpos, d.site(name).xmat.reshape(3, 3)
            worst_p = max(worst_p, np.linalg.norm(pos - pose[:3, 3]))
            worst_r = max(worst_r, np.linalg.norm(rot - pose[:3, :3]))
    check(worst_p < 1e-6 and worst_r < 1e-6,
          f"{len(poses)} links and tool frames over 10 random postures "
          f"(worst {worst_p:.1e} m, {worst_r:.1e})")
    robot.reset()


def ground(robot, task):
    print("\nground: wheels on task.yaml's floor")
    params = task["pick_place_task"]["ros__parameters"]
    m, d = robot.model, robot.data
    lowest = np.inf
    for g in range(m.ngeom):
        if m.geom_group[g] == 2 and "wheel" in m.body(m.geom_bodyid[g]).name:
            mesh = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh] + m.mesh_vertnum[mesh]]
            lowest = min(lowest, (v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])[:, 2].min())
    check(abs(lowest - params["ground_z"]) < 5e-4,
          f"wheel bottom z={lowest:.4f}, task.yaml ground_z={params['ground_z']}")
    check(abs(robot.wheel_radius - params["base"]["wheel_radius"]) < 5e-4,
          f"wheel radius {robot.wheel_radius:.4f}, task.yaml {params['base']['wheel_radius']}")
    check(d.ncon == 0, f"no contacts at 'home' before anything moves ({d.ncon})")


def gravity(robot):
    print("\ngravity: every keyframe holds within the effort limits")
    m, d = robot.model, robot.data
    for k in range(m.nkey):
        robot.reset(m.key(k).name)
        load = {m.joint(j).name: abs(d.qfrc_bias[m.jnt_dofadr[j]]) / m.jnt_actfrcrange[j][1]
                for j in range(m.njnt) if m.jnt_actfrclimited[j]}
        worst = max(load, key=load.get)
        q0 = d.qpos.copy()
        robot.step(3.0)
        drift = np.abs(d.qpos - q0)[:m.jnt_qposadr[m.joint("workpiece").id]]
        check(load[worst] < 1 and drift.max() < 2e-3,
              f"{m.key(k).name}: worst static load {load[worst]:.0%} of effort ({worst}), "
              f"drift after 3 s {drift.max() * 1000:.2f} mrad/mm")
    robot.reset()


def gripper(robot):
    print("\ngripper: 95 mm open, 0 mm closed")
    for side in ("left", "right"):
        gaps = []
        for gap in (0.095, 0.0):
            robot.set_gripper(side, gap)
            robot.step(1.0)
            body = robot.data
            f1 = body.body(f"{side}_finger01_Link").xpos
            f2 = body.body(f"{side}_finger02_Link").xpos
            gaps.append(np.linalg.norm(f1 - f2))
        check(abs(gaps[0] - 0.095) < 5e-4 and gaps[1] < 5e-4,
              f"{side}: open {gaps[0] * 1000:.1f} mm, closed {gaps[1] * 1000:.1f} mm")
    robot.reset()


def base(robot):
    print("\nbase: holonomic drive and swerve modules")
    steering = [f"{n}_steering_joint" for n in ("left_front", "right_front", "rear")]
    robot.drive(0.0, 0.3, 0.0)  # pure strafe left: every module at +90 deg
    robot.step(1.0)
    angles = robot.joints(steering)
    check(np.allclose(np.abs(angles), math.pi / 2, atol=0.02),
          f"strafing: modules steer {np.degrees(angles).round(1)} deg")
    robot.drive(0.0, 0.0, 0.5)  # spin: modules tangential to the centre
    robot.step(1.0)
    ok = True
    for name, ((px, py), _) in robot._modules.items():
        tangent = math.atan2(px, -py)
        angle = robot.joint(f"{name}_steering_joint")
        ok &= abs(math.sin(angle - tangent)) < 0.03
    check(ok, "spinning: every module tangential to the base centre")
    goal = (-0.8, -0.6, 1.2)  # back into the aisle, clear of the bench
    robot.drive_to(*goal)
    robot.step(0.5)
    pose = robot.base_pose()
    error = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
    check(error < 2e-3 and abs(pose[2] - goal[2]) < 2e-3,
          f"drive_to{goal}: {error * 1000:.1f} mm, {math.degrees(pose[2] - goal[2]):+.2f} deg")
    # Push 0.9 m worth into the bench, then stop: the chassis rolls under the
    # top until the body meets its edge, and must not lunge once released.
    robot.reset()
    robot.drive(0.3, 0.0, 0.0)
    robot.step(3.0)
    blocked = robot.base_pose()[0]
    robot.drive(0.0, 0.0, 0.0)
    robot.step(1.0)
    moved = abs(robot.base_pose()[0] - blocked)
    check(blocked < 0.3 and moved < 0.06,
          f"blocked by the bench at x={blocked:.3f} m, moves {moved * 1000:.0f} mm once stopped")
    robot.reset()


def cameras(robot):
    print("\ncameras: upright, looking where the lenses point")
    m, d = robot.model, robot.data
    robot.reset("look_down")
    for name in ("head_d435", "body_d435"):
        rot = d.cam_xmat[m.camera(name).id].reshape(3, 3)
        right, down = rot[:, 0], -rot[:, 1]  # optical x, y
        check(down @ [0, 0, -1] > 0.5 and right @ [0, -1, 0] > 0.9,
              f"{name}: image right = robot right ({right @ [0, -1, 0]:.2f}), "
              f"image down = world down ({down @ [0, 0, -1]:.2f})")
    for side in ("left", "right"):
        name = f"{side}_wrist_d405"
        c = m.camera(name).id
        mid = (d.body(f"{side}_finger01_Link").xpos + d.body(f"{side}_finger02_Link").xpos) / 2
        local = d.cam_xmat[c].reshape(3, 3).T @ (mid - d.cam_xpos[c])
        height = m.cam_resolution[c][1]
        focal = height / 2 / math.tan(math.radians(m.cam_fovy[c]) / 2)
        v = height / 2 - focal * local[1] / -local[2]
        check(local[2] < 0 and v > height / 2,
              f"{name}: the fingers are in front and in the lower half (row {v:.0f}/{height})")
    try:
        stds = {n: robot.render(n).std() for n in
                ("head_d435", "body_d435", "left_wrist_d405", "right_wrist_d405")}
        check(all(s > 5 for s in stds.values()),
              "all four cameras render a picture: " + ", ".join(f"{n} std {s:.0f}"
                                                                for n, s in stds.items()))
    except Exception as error:  # no GL: say so, do not pass silently
        check(False, f"rendering unavailable ({error}); try MUJOCO_GL=egl")
    robot.reset()


def scan(robot):
    print("\nscan: the 2D lidar sees the world")
    m, d = robot.model, robot.data
    angles, ranges = robot.scan()
    origin = d.site("2Dlidar").xpos
    legs = [d.geom_xpos[g] for g in range(m.ngeom)
            if m.geom_bodyid[g] == m.body("bench").id and m.geom(g).name != "bench_top"]
    nearest = min(np.linalg.norm((leg - origin)[:2]) for leg in legs)
    hits = ranges[np.isfinite(ranges)]
    check(len(angles) == 819 and hits.size and abs(hits.min() - nearest) < 0.05,
          f"{len(angles)} beams, nearest return {hits.min():.3f} m, nearest bench leg "
          f"centre {nearest:.3f} m")


def pick(robot):
    print("\npick: demo.py's pick-and-place, by physics")
    result = demo.pick_and_place(robot)
    check(abs(result["gap"] - 0.07) < 2e-3, f"jaws stop on the 70 mm part ({result['gap'] * 1000:.1f} mm)")
    check(result["lifted"] > 0.1, f"part lifted {result['lifted'] * 1000:.0f} mm")
    check(result["carried"] < 0.01, f"part stays in hand while the base strafes "
                                     f"({result['carried'] * 1000:.1f} mm from the TCP)")
    error = np.linalg.norm(result["placed"] - result["expected"])
    check(error < 0.01, f"placed {error * 1000:.1f} mm from where it was meant to go")


def main():
    task = yaml.safe_load(convert_urdf.DEFAULT_TASK.read_text())
    with Franzi() as robot:
        m = robot.model
        print(f"model: {m.nbody} bodies, {m.njnt} joints, {m.nu} actuators, "
              f"{m.ncam} cameras, robot mass {m.body_subtreemass[m.body('base_link').id]:.1f} kg")
        conversion()
        kinematics(robot)
        ground(robot, task)
        gravity(robot)
        gripper(robot)
        base(robot)
        cameras(robot)
        scan(robot)
        pick(robot)
    print(f"\n{'all checks passed' if not failures else f'{len(failures)} FAILED'}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
