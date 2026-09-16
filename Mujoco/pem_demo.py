#!/usr/bin/env python3
"""PEM cell demo: carry every electrode stack from the side table into its tray.

For each carrier on the table (pem_cell.py's scene): drive to it, take its
handle, lift it above tray height, drive to the anode or cathode tray, bring
it in along the tray's x direction above the corner guides, lower it until the
tine sits on the tray floor (or on the stack already there), open the clamp,
draw the tine out along x from under the stack, and take the empty carrier
back to its place on the table. The second stack of a kind lands on the first.

Everything is physics: the gripper holds the handle by friction, the clamp is
the carrier's own actuator, the stack is a free body that stays in the tray
because the corner guides stop it when the tine slides out from under it.

    Mujoco/.venv/bin/python Mujoco/pem_demo.py                 # MuJoCo viewer
    Mujoco/.venv/bin/python Mujoco/pem_demo.py --headless      # just run it
    MUJOCO_GL=egl Mujoco/.venv/bin/python Mujoco/pem_demo.py --record out.mp4
"""

import argparse
import math
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
import yaml

import demo
import franzi
import pem_cell
from franzi import Franzi, min_jerk, top_down

SCENE = Path(__file__).resolve().parent / "model" / "pem_scene.xml"
SIDE = demo.SIDE
GRASP_DEPTH = demo.GRASP_DEPTH  # grasp frame below the handle's top
# The hand closes across the handle's 20 mm along the cell's x: the loaded
# carrier's weight then sits on a friction couple between the two fingers
# rather than on their torsional friction. It keeps this orientation for the
# whole run.
GRASP = top_down(-math.pi / 2)
# Where the hand is, in base_link, whenever the base moves: in front and high
# enough that a held carrier's tine clears the trays' tops.
READY = np.array([0.25, 0.16, 1.12])
CLEAR = 0.010    # m the tine clears the guide tops while the carrier moves in
RELEASE = 0.001  # m the tine hovers over its support when the clamp opens
LIFT = 0.002     # m up off the tray floor before the tine is drawn out
WITHDRAW = 0.21  # m the carrier moves along +x to draw the tine out
APPROACH = 0.12  # m above a handle / the table spot before going down


def log(robot, message):
    print(f"[{robot.time:6.1f}s] {message}", flush=True)


class Cell:
    """The scene's frame and stations as the robot needs them."""

    def __init__(self, robot):
        self.robot = robot
        self.layout = pem_cell.Cell(yaml.safe_load(pem_cell.CONFIG.read_text()))
        d, m = robot.data, robot.model
        self.ex = d.body("frame").xmat.reshape(3, 3)[:, 0]  # cell x in the world
        self.guide_top = (pem_cell.TRAY_SIZE[2] - pem_cell.TRAY_CROWN) * pem_cell.MM
        self.entry = self.layout.config["dock"]["entry_offset"] * pem_cell.MM
        self.filled = {"anode_tray": 0.0, "cathode_tray": 0.0}  # stack height in each
        docks = [m.site(s).name for s in range(m.nsite) if m.site(s).name.endswith("_dock")]
        # The lane the base travels along between stations: 10 cm behind the
        # rearmost dock, clear of the table's and the frame's front edges.
        self.lane = min(d.site(n).xpos[0] for n in docks) - 0.10

    def dock(self, station):
        s = self.robot.data.site(f"{station}_dock")
        return s.xpos[0], s.xpos[1], math.atan2(s.xmat[3], s.xmat[0])


def ready_point(robot):
    x, y, yaw = robot.base_pose()
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([x + c * READY[0] - s * READY[1], y + s * READY[0] + c * READY[1], READY[2]])


def drive(robot, station, cell, callback):
    """Back onto the lane, along it, then straight in to the dock."""
    log(robot, f"driving to {station}")
    x, y, yaw = cell.dock(station)
    here = robot.base_pose()
    if math.hypot(here[0] - x, here[1] - y) > 0.02:
        robot.drive_to(cell.lane, here[1], here[2], callback=callback)
        robot.drive_to(cell.lane, y, yaw, callback=callback)
    robot.drive_to(x, y, yaw, callback=callback)
    robot.step(0.3, callback)


def line(robot, target, duration, callback):
    """Straight line of the grasp frame."""
    robot.move_tcp(SIDE, target, GRASP, duration, callback=callback)


def carry(robot, carrier, offset, target, duration, callback):
    """Straight line of the held carrier's frame to ``target``."""
    line(robot, np.asarray(target) + offset, duration, callback)


def grip_offset(robot, carrier):
    """Grasp frame minus carrier frame in the world, as the hand holds it now
    (the orientation stays fixed)."""
    return robot.tcp_pose(SIDE)[0] - robot.data.body(carrier).xpos


def raise_arm(robot, callback):
    """From the hanging arm to READY, where nothing is in reach."""
    solution, ok = robot.solve_ik(SIDE, ready_point(robot), GRASP)
    if not ok:
        raise RuntimeError("READY is out of reach")
    robot.move_joints(solution, 2.5, callback)


def pick(robot, carrier, callback):
    """From READY over the handle, down, grip; up to carrying height, back to READY."""
    grasp = robot.data.site(f"{carrier}_handle").xpos - [0, 0, GRASP_DEPTH]
    robot.set_gripper(SIDE, 0.095)
    line(robot, grasp + [0, 0, APPROACH], 2.0, callback)
    line(robot, grasp, 1.2, callback)
    robot.set_gripper(SIDE, 0.0)
    robot.step(0.8, callback)
    log(robot, f"holding {carrier}'s handle, jaw gap {robot.gripper_gap(SIDE) * 1000:.1f} mm")
    line(robot, [grasp[0], grasp[1], READY[2]], 1.5, callback)
    line(robot, ready_point(robot), 1.0, callback)


def servo(robot, stack, xy, path, duration, callback, period=0.05, gain=0.3):
    """Follow a planned hand ``path`` ([{joint: value}], waypoints of a
    vertical line) while holding the stack's centre on ``xy``: every
    ``period`` a share ``gain`` of the stack's measured x, y error is added
    to the hand's command (an integrator; the arm lags a step behind, so a
    gain near 1 oscillates), solved from the path's own joints so the arm
    stays on the planned IK branch. The anode stack fills the tray pocket's
    width to 0.25 mm a side, so open-loop IK (0.5 mm tolerance) and the arm's
    sag under the load would land it on the guides."""
    steps = max(1, round(duration / period))
    joints = robot.arm_joints(SIDE)
    heights = []
    for q in path:  # the grasp frame's height at each waypoint
        scratch = robot.data.qpos.copy()
        for j, v in q.items():
            robot.data.qpos[robot.model.jnt_qposadr[robot.model.joint(j).id]] = v
        mujoco.mj_kinematics(robot.model, robot.data)
        heights.append(robot.data.site(f"{SIDE}_{franzi.TOOL}").xpos[2])
        robot.data.qpos[:] = scratch
    mujoco.mj_forward(robot.model, robot.data)
    command = robot.tcp_pose(SIDE)[0][:2].copy()
    z0 = robot.tcp_pose(SIDE)[0][2]
    for k in range(1, steps + 1):
        command += gain * (np.asarray(xy) - robot.data.body(stack).xpos[:2])
        z = z0 + (heights[-1] - z0) * min_jerk(k / steps)
        seed = path[int(np.argmin(np.abs(np.array(heights) - z)))]
        solution, ok = robot.solve_ik(SIDE, [*command, z], GRASP, seed=[seed[j] for j in joints],
                                      restarts=0, tolerance=(2e-5, 1e-3))
        if not ok:
            raise RuntimeError(f"no IK holding {stack} over {np.round(xy, 4)}")
        robot.move_joints(solution, period, callback)
    return robot.data.body(stack).xpos[:2] - xy


def follow(robot, path, duration, callback):
    for q in path:
        robot.move_joints(q, duration / len(path), callback)


def insert(robot, cell, carrier, stack, tray, callback):
    """In along -x above the guides, down, clamp open, tine out along +x.

    The whole path is planned before the carrier moves, as one chain of
    straight lines on one IK branch: the arm is redundant, and a branch that
    reaches the pocket can run the wrist into its limit on the way out."""
    geo = cell.layout.carrier(stack.split("_")[0])
    floor = robot.data.site(tray).xpos.copy()
    high = floor + [0, 0, cell.guide_top + CLEAR + geo["tine"]]
    low = floor + [0, 0, cell.filled[tray] + geo["tine"] + RELEASE]
    offset = grip_offset(robot, carrier)
    points = [high + cell.ex * cell.entry, high, low, low + [0, 0, LIFT],
              low + [0, 0, LIFT] + cell.ex * WITHDRAW, high + cell.ex * WITHDRAW]
    legs, start = [], robot.tcp_pose(SIDE)[0]
    seed = robot.targets(robot.arm_joints(SIDE))
    for point in points:
        leg = robot.plan_line(SIDE, start, point + offset, GRASP, seed)
        if leg is None:
            raise RuntimeError(f"{carrier}: no IK path into {tray} through {np.round(point, 3)}")
        legs.append(leg)
        start, seed = point + offset, [leg[-1][j] for j in robot.arm_joints(SIDE)]
    follow(robot, legs[0], 2.0, callback)
    log(robot, f"{carrier}: sliding in along -x above {tray}'s guides")
    follow(robot, legs[1], 2.5, callback)
    # Settle over the pocket, then down with the stack held on its centre.
    pocket = robot.data.site(tray).xpos[:2]
    servo(robot, stack, pocket, legs[1][-1:], 1.5, callback)
    error = servo(robot, stack, pocket, legs[2], 3.0, callback)
    log(robot, f"{stack}: down in {tray}, {np.abs(error).max() * 1000:.2f} mm off its centre")
    robot.set_targets({f"{carrier}_clamp": geo["open"]})
    robot.step(0.6, callback)
    log(robot, f"{carrier}: clamp open, drawing the tine out along +x")
    follow(robot, legs[3], 0.3, callback)
    follow(robot, legs[4], 3.0, callback)
    follow(robot, legs[5], 1.0, callback)
    cell.filled[tray] += geo["height"]
    line(robot, ready_point(robot), 1.5, callback)


def put_back(robot, carrier, home, callback):
    """Set the empty carrier down where it came from, let go, back to READY."""
    offset = grip_offset(robot, carrier)
    carry(robot, carrier, offset, home + [0, 0, APPROACH], 2.0, callback)
    carry(robot, carrier, offset, home + [0, 0, 0.001], 1.5, callback)
    robot.set_gripper(SIDE, 0.095)
    robot.step(0.6, callback)
    grasp = robot.tcp_pose(SIDE)[0]
    line(robot, grasp + [0, 0, APPROACH], 1.0, callback)
    line(robot, ready_point(robot), 1.5, callback)


def transfer(robot, cell, carrier, stack, tray, callback=None):
    drive(robot, carrier, cell, callback)
    home = robot.data.body(carrier).xpos.copy()
    pick(robot, carrier, callback)
    drive(robot, tray, cell, callback)
    insert(robot, cell, carrier, stack, tray, callback)
    drive(robot, carrier, cell, callback)
    put_back(robot, carrier, home, callback)
    log(robot, f"{carrier} back on the table")


def run(robot, callback=None):
    """Every stack into its tray. Returns, for check_model.py, {"stacks":
    {stack: (tray, offset from the tray floor centre in the tray frame,
    tilt in degrees)}, "carriers": {carrier: distance from where it
    started}}."""
    cell = Cell(robot)
    start = robot.base_pose()
    homes = {c: robot.data.body(c).xpos.copy() for c, *_ in cell.layout.carriers}
    robot.set_posture("head", "look_down")
    raise_arm(robot, callback)
    order = sorted(cell.layout.carriers, key=lambda c: (c[2] != "anode", c[0]))
    for carrier, stack, kind, _, _ in order:
        transfer(robot, cell, carrier, stack, f"{kind}_tray", callback)
    # Back where it started, clear of everything, and the arm down.
    log(robot, "driving back to the start")
    here = robot.base_pose()
    robot.drive_to(cell.lane, here[1], here[2], callback=callback)
    robot.drive_to(*start, callback=callback)
    robot.move_joints({j: 0.0 for j in robot.arm_joints(SIDE)}, 2.5, callback)
    robot.step(1.0, callback)
    result = {"stacks": {}, "carriers": {}}
    d = robot.data
    for carrier, stack, kind, _, _ in order:
        tray = f"{kind}_tray"
        rot = d.body(tray).xmat.reshape(3, 3)
        local = rot.T @ (d.body(stack).xpos - d.site(tray).xpos)
        tilt = math.degrees(math.acos(min(1.0, d.body(stack).xmat[8])))
        result["stacks"][stack] = (tray, local, tilt)
        result["carriers"][carrier] = np.linalg.norm(d.body(carrier).xpos - homes[carrier])
        log(robot, f"{stack} in {tray} at {np.round(local * 1000, 1)} mm, tilt {tilt:.2f} deg; "
                   f"{carrier} {result['carriers'][carrier] * 1000:.1f} mm from its place")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--headless", action="store_true", help="no window, just run")
    mode.add_argument("--record", type=Path, metavar="MP4", help="render to an H.264 video")
    parser.add_argument("--camera", default="overview", help="camera to record")
    args = parser.parse_args()

    robot = Franzi(SCENE)
    if args.headless:
        run(robot)
    elif args.record:
        recorder = demo.Recorder(robot, args.record, args.camera)
        try:
            run(robot, recorder)
        finally:
            recorder.close()
            robot.close()
        print(f"wrote {args.record}")
    else:
        import mujoco.viewer

        with mujoco.viewer.launch_passive(robot.model, robot.data) as viewer:
            wall = time.monotonic() - robot.time

            def sync(r):
                if round(r.time / r.model.opt.timestep) % 8:
                    return
                if not viewer.is_running():
                    sys.exit(0)
                viewer.sync()
                time.sleep(max(0.0, r.time - (time.monotonic() - wall)))

            run(robot, sync)
            while viewer.is_running():
                robot.step(1 / 60)
                viewer.sync()
                time.sleep(1 / 60)


if __name__ == "__main__":
    main()
