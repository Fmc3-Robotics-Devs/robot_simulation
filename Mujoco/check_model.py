#!/usr/bin/env python3
"""Check the MuJoCo model against the rest of the repository and physics.

Each section tests one claim the README makes, against an independent
reference where there is one:

* merge       - franzi_merged/ is merge_urdf.py's current output; its links sit
                where the export they came from puts them (franzi_description
                up to the elbows, wheel_robot_7.24 from the wrists on); the SDK
                flange/TCP are franzi_description's; the grasp centre is
                task.yaml's tcp_offset
* kinematics  - MuJoCo FK equals the merged URDF's, link by link
* ground      - wheels touch the floor at task.yaml's ground_z, radius matches
* gravity     - every keyframe holds still within the joint effort limits
* gripper     - 95 mm open, 0 mm closed, both sides
* base        - drive_to lands, the swerve modules steer along the motion
* cameras     - the D435 images are upright, the wrist cameras see the fingers
* scan        - the 2D lidar sees the bench legs where they are
* pick        - demo.py's pick-and-place actually moves the part by physics
* pem cell    - model/pem_scene.xml: the frame and trays where the measurement
                drawing puts them, stacks the size of the sheet drawings, the
                carriers fit the trays; pem_demo.py moves every stack into its
                tray by physics and touches nothing else

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
import merge_urdf  # noqa: E402
import pem_cell as pem  # noqa: E402
import pem_demo  # noqa: E402
from franzi import Franzi, top_down  # noqa: E402

# The ROS stack's description: wheel_robot_26.8.16_3 with the REP-103 root and
# the SDK flange/TCP frames appended.
DESCRIPTION_URDF = (HERE.parent / "Rviz" / "src" / "franzi_description" / "urdf"
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


def zero_axes(path):
    """{joint: axis in base_link} at joint zero."""
    poses = urdf_fk(path, {})
    return {j.get("name"): poses[j.find("child").get("link")][:3, :3]
            @ np.array([float(v) for v in j.find("axis").get("xyz").split()])
            for j in ET.parse(path).getroot().findall("joint") if j.get("type") != "fixed"}


def merge(robot, task):
    print("\nmerge: franzi_merged/ against the two exports")
    text, meshes = merge_urdf.build()
    out = merge_urdf.OUT
    check(text == (out / "urdf" / f"{merge_urdf.NAME}.urdf").read_text(),
          "the URDF is merge_urdf.py's current output")
    stale = [n for n, src in meshes.items()
             if not (out / "meshes" / n).exists() or (out / "meshes" / n).read_bytes() != src.read_bytes()]
    extra = sorted({p.name for p in (out / "meshes").glob("*.STL")} - meshes.keys())
    check(not stale and not extra, f"{len(meshes)} meshes are byte copies of their sources"
          + (f" (stale {stale}, extra {extra})" if stale or extra else ""))

    # Which export each merged frame must match, and under which name there.
    merged = convert_urdf.DEFAULT_URDF
    wrist = merge_urdf.Export(merge_urdf.WRIST_URDF)
    to_wrist = {}  # merged name -> 7.24 name, grafted links and their joints
    for side in merge_urdf.SIDES:
        for link in wrist.subtree(f"{side}_wrist_yaw_Link"):
            for name in (link, wrist.joints[link].get("name")):
                to_wrist[merge_urdf.renamed(name, side)] = name
    references = (
        ("chassis to elbows + SDK flange/TCP", DESCRIPTION_URDF, {},
         lambda link: link not in to_wrist and not link.endswith("_grasp")),
        ("wrists and grippers", merge_urdf.WRIST_URDF, to_wrist, lambda link: link in to_wrist),
    )
    m = robot.model
    axes = zero_axes(merged)
    rng = np.random.default_rng(2)
    for what, path, names, compared in references:
        # The exports disagree on joint signs, so map each joint by the sign
        # of its axis against the reference's at zero (prismatic fingers of
        # the other gripper are perpendicular and not needed).
        ref_axes = zero_axes(path)
        signs = {j: float(np.round(a @ ref_axes[names.get(j, j)])) for j, a in axes.items()
                 if names.get(j, j) in ref_axes and abs(a @ ref_axes[names.get(j, j)]) > 0.99}
        worst_p = worst_r = 0.0
        frames = set()
        for _ in range(10):
            q = {j: rng.uniform(*(m.joint(j).range if m.jnt_limited[m.joint(j).id]
                                  else (-math.pi, math.pi))) for j in axes}
            ours = urdf_fk(merged, q)
            theirs = urdf_fk(path, {names.get(j, j): signs[j] * v for j, v in q.items() if j in signs})
            for link, pose in ours.items():
                if not compared(link):
                    continue
                frames.add(link)
                other = theirs[names.get(link, link)]
                worst_p = max(worst_p, np.linalg.norm(pose[:3, 3] - other[:3, 3]))
                worst_r = max(worst_r, np.abs(pose[:3, :3] - other[:3, :3]).max())
        flipped = sorted(j.replace("_joint", "") for j, sign in signs.items()
                         if sign < 0 and j.startswith("left"))
        check(worst_p < 5e-5 and worst_r < 1e-4,
              f"{what}: {len(frames)} frames on {path.parent.parent.name}'s over 10 random "
              f"postures (worst {worst_p * 1000:.3f} mm, {worst_r:.1e}); turning the other way "
              f"there: {', '.join(flipped) or 'nothing'} (and right)")

    p = task["pick_place_task"]["ros__parameters"]
    poses = urdf_fk(merged, {})
    grasp = np.linalg.inv(poses[p["tip_link"]]) @ poses[p["tip_link"].replace("wrist_roll_Link", "grasp")]
    error = np.linalg.norm(grasp[:3, 3] - p["tcp_offset"])
    check(error < 3e-4, f"grasp centre {np.round(grasp[:3, 3], 4)} in {p['tip_link']} = task.yaml "
                        f"tcp_offset {p['tcp_offset']} ({error * 1000:.2f} mm)")


def kinematics(robot):
    print("\nkinematics: MuJoCo FK = merged URDF FK")
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
        poses = urdf_fk(convert_urdf.DEFAULT_URDF, positions)
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
        # The midpoint between the open fingers projects just past the bottom
        # edge; the finger pads themselves show in the two bottom corners.
        check(local[2] < 0 and v > height / 2,
              f"{name}: the fingers are in front and below the image centre "
              f"(their midpoint at row {v:.0f} of {height})")
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
    check(result["slip"] < 0.005, f"part stays in hand while the base strafes "
                                  f"(slips {result['slip'] * 1000:.1f} mm)")
    error = np.linalg.norm(result["placed"] - result["expected"])
    check(error < 0.01, f"placed {error * 1000:.1f} mm from where it was meant to go")


def pem_cell():
    print("\npem cell: model/pem_scene.xml against pem_cell.yaml, and pem_demo.py by physics")
    config = yaml.safe_load(pem.CONFIG.read_text())
    t, mm = config["trays"], 1e-3
    with Franzi(HERE / "model" / "pem_scene.xml") as robot:
        m, d = robot.model, robot.data
        frame = d.body("frame")
        floor = m.geom("floor").pos[2]
        to_cell = frame.xmat.reshape(3, 3).T

        def span(geom, axis):  # a box geom's (low, high) along a cell axis, mm
            g = d.geom(geom)
            centre = to_cell @ (g.xpos - frame.xpos)
            half = np.abs(to_cell @ g.xmat.reshape(3, 3)) @ m.geom(geom).size
            return (centre[axis] - half[axis]) / mm, (centre[axis] + half[axis]) / mm

        anode, cathode = span("anode_tray_base", 0), span("cathode_tray_base", 0)
        assembly = span("assembly_tray", 0)
        gaps = (cathode[0] - anode[1], assembly[0] - anode[1], cathode[0] - assembly[1])
        measured = (t["inner_gap_anode_cathode"], t["inner_gap_anode_assembly"],
                    t["inner_gap_assembly_cathode"])
        check(np.allclose(gaps, measured, atol=0.5),
              "inner x gaps anode-cathode / anode-assembly / assembly-cathode "
              f"{' / '.join(f'{g:.1f}' for g in gaps)} mm, measured {' / '.join(map(str, measured))}")
        heights = [(d.body(n).xpos[2] - floor) / mm for n in ("anode_tray", "cathode_tray", "assembly_tray")]
        expected = [t["height_anode"], t["height_cathode"], t["height_assembly"]]
        check(np.allclose(heights, expected, atol=0.5),
              f"tray undersides {' / '.join(f'{h:.1f}' for h in heights)} mm above the floor, "
              f"measured {' / '.join(map(str, expected))}")
        size = [np.diff(span("assembly_tray", a))[0] for a in (0, 1)]
        behind = np.mean(span("assembly_tray", 1)) - np.mean(span("anode_tray_base", 1))
        check(np.allclose(size, [measured[0] - measured[1] - measured[2], t["assembly_depth"]])
              and abs(behind - t["assembly_centre_offset"]) < 0.5,
              f"assembly tray {size[0]:.0f} x {size[1]:.0f} mm (1175 - 420 - 550 wide), centreline "
              f"{behind:.1f} mm behind the others' (config {t['assembly_centre_offset']})")
        ok = abs(span("beam", 2)[0] - floor / mm) < 0.5
        for tray in ("anode_tray", "cathode_tray", "assembly_tray"):
            post = f"{tray}_post"
            ok &= np.allclose([np.diff(span(post, a))[0] for a in (0, 1)], config["profile"])
            ok &= abs(np.mean(span(post, 0)) - np.mean(span(f"{tray}_base" if tray != "assembly_tray"
                                                             else tray, 0))) < 0.5
            ok &= abs(span(post, 2)[1] - (d.body(tray).xpos[2]) / mm) < 0.5
        check(ok, f"{config['profile']} x {config['profile']} profile: beam on the floor, a post "
                  "under each tray (centred in x) up to its underside")

        # Stacks: the sheet stacks of the drawings, each in a carrier.
        stacks = [m.body(b).name for b in range(m.nbody) if "_stack_" in m.body(b).name]
        carriers = [s.replace("stack", "carrier") for s in stacks]
        ok = True
        for name in stacks:
            sheet = config["sheets"][name.split("_")[0]]
            height = sheet["thickness"] * config["stacks"]["sheets"]
            body = 2 * m.geom(f"{name}_body").size / mm
            tab = 2 * m.geom(f"{name}_tab").size / mm
            ok &= np.allclose(body, [*sheet["body"], height]) and np.allclose(tab, [*sheet["tab"], height])
        kinds = {k: sum(n.startswith(k) for n in stacks) for k in ("anode", "cathode")}
        mass = {n: m.body_subtreemass[m.body(n).id] for n in ("anode_stack_1", "cathode_stack_1",
                                                               "anode_carrier_1")}
        check(ok and kinds == config["stacks"]["count"],
              f"{kinds['anode']} anode + {kinds['cathode']} cathode stacks of "
              f"{config['stacks']['sheets']} sheets, sized as drawn (anode {mass['anode_stack_1']:.2f} "
              f"kg, cathode {mass['cathode_stack_1']:.2f} kg, carrier {mass['anode_carrier_1']:.2f} kg)")

        # The carrier's tine, pad and arm pass the gap between a tray's guides.
        tray = m.body("anode_tray").id
        guides = [g for g in range(m.ngeom) if m.geom_bodyid[g] == tray and m.geom_group[g] == 3
                  and m.geom(g).name != "anode_tray_base"]
        faces = [np.abs(m.geom_pos[g][:2]) / mm - m.geom_size[g][:2] / mm for g in guides]
        room = {}
        for kind in ("anode", "cathode"):
            half = np.array(config["sheets"][kind]["body"]) / 2
            # Along x only the guides the body overlaps in y stop it, and so on.
            room[kind] = [2 * min(f[a] for f in faces if f[1 - a] < half[1 - a] - 1e-6) for a in (0, 1)]
        end_gap = 2 * min(f[1] for f in faces if f[0] > 90)  # between the guides at an end
        widths = [2 * m.geom(f"anode_carrier_1_{p}").size[1] / mm for p in ("tine", "arm", "pad")]
        check(all(np.all(np.array(config["sheets"][k]["body"]) <= np.array(r) + 1e-6)
                  for k, r in room.items()) and max(widths) < end_gap,
              "tray guides leave " + ", ".join(f"{r[0]:.1f} x {r[1]:.1f} mm for the {k} body "
                                                f"{config['sheets'][k]['body']}" for k, r in room.items())
              + f"; the carrier ({max(widths):.0f} mm) passes their {end_gap:.1f} mm end gap")

        # At rest: nothing penetrates, the clamps hold, nothing moves.
        start = {n: d.body(n).xpos.copy() for n in stacks + carriers}
        touching = sum(c.dist < -1e-3 for c in d.contact[:d.ncon])
        robot.step(2.0)
        moved = max(np.linalg.norm(d.body(n).xpos - start[n]) for n in start)
        force = min(abs(d.actuator(f"{c}_clamp").force[0]) for c in carriers)
        check(touching == 0 and moved < 1e-3,
              f"no penetrations at the start ({touching}); stacks and carriers settle "
              f"{moved * 1000:.2f} mm in 2 s, every clamp pressing {force:.0f} N")

        # The demo, by physics: every stack into its tray, every carrier
        # back, and the robot touches nothing but the handles on the way.
        robot.reset()
        stray = set()

        def watch(r):
            for c in d.contact[:d.ncon]:
                bodies = [m.body(m.geom_bodyid[g]) for g in (c.geom1, c.geom2)]
                robot_side = [b for b in bodies if b.rootid == m.body("base_link").id]
                if robot_side and not all("finger" in b.name for b in robot_side):
                    stray.add(tuple(sorted(b.name for b in bodies)))
                elif robot_side and not any("carrier" in b.name for b in bodies):
                    stray.add(tuple(sorted(b.name for b in bodies)))

        with contextlib.redirect_stdout(io.StringIO()):
            result = pem_demo.run(robot, watch)
        worst = {"xy": 0.0, "z": 0.0, "tilt": 0.0}
        layer = {}
        for stack, (tray, local, tilt) in sorted(result["stacks"].items()):
            kind = stack.split("_")[0]
            h = config["sheets"][kind]["thickness"] * config["stacks"]["sheets"] * mm
            below = layer.get(tray, 0.0)
            layer[tray] = below + h
            pocket = (np.array(room[kind]) - config["sheets"][kind]["body"]) / 2 * mm
            worst["xy"] = max(worst["xy"], *(np.abs(local[:2]) - pocket))
            worst["z"] = max(worst["z"], abs(local[2] - below - h / 2))
            worst["tilt"] = max(worst["tilt"], tilt)
        check(worst["xy"] <= 1e-4 and worst["z"] < 1e-3 and worst["tilt"] < 0.5,
              f"pem_demo.py puts all {len(result['stacks'])} stacks in their trays, two layers "
              f"each: inside the guides, {worst['z'] * 1000:.2f} mm off their layer height, "
              f"tilt {worst['tilt']:.2f} deg or less")
        back = max(result["carriers"].values())
        check(back < 5e-3 and not stray,
              f"every carrier back within {back * 1000:.1f} mm of its place; the robot touched "
              f"nothing but the handles ({sorted(stray) or 'none'})")


def main():
    task = yaml.safe_load(convert_urdf.DEFAULT_TASK.read_text())
    with Franzi() as robot:
        m = robot.model
        print(f"model: {m.nbody} bodies, {m.njnt} joints, {m.nu} actuators, "
              f"{m.ncam} cameras, robot mass {m.body_subtreemass[m.body('base_link').id]:.1f} kg")
        merge(robot, task)
        kinematics(robot)
        ground(robot, task)
        gravity(robot)
        gripper(robot)
        base(robot)
        cameras(robot)
        scan(robot)
        pick(robot)
    pem_cell()
    print(f"\n{'all checks passed' if not failures else f'{len(failures)} FAILED'}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
