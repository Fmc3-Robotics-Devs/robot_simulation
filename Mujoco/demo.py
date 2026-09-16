#!/usr/bin/env python3
"""Pick-and-place demo: drive to the bench, pick the workpiece, move it.

The robot starts back in the aisle, drives (strafing and turning, so the
swerve modules have something to do) to the dock pose task.yaml defines, looks
down, raises its left hand short of the bench edge, slides it over the part,
picks the part top-down, strafes 25 cm along the bench carrying it, sets it
down there and backs its arm out the way it came.

The grasp is physical: the part is a free body held only by finger friction.
Its pose is read from the simulator (ground truth, like the ROS stack's mock
detector) - there is no perception in this loop.

    Mujoco/.venv/bin/python Mujoco/demo.py                    # MuJoCo viewer
    Mujoco/.venv/bin/python Mujoco/demo.py --headless         # just run it
    MUJOCO_GL=egl Mujoco/.venv/bin/python Mujoco/demo.py --record out.mp4
"""

import argparse
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from franzi import Franzi, top_down

START = (-1.0, -0.6, -0.5)    # odom x, y, yaw the robot starts from
DOCK = (0.0, 0.0, 0.0)        # the pose task.yaml's dock offset is measured from
STRAFE = 0.25                 # m along the bench to carry the part
APPROACH = 0.12               # m above the grasp to come down from
# How far short of the part (towards the robot) the hand rises to approach
# height. Joint-space moves are not collision-checked: straight from the
# hanging arm to above the part, the hand sweeps under the bench top's edge.
BACKOFF = 0.2
# Where the grasp frame (between the finger pads) sits below the part's top
# face: the palm's collision hull ends 8 mm above it and the fingertips 17 mm
# below, so 5 mm down grips the part over 22 mm of finger pad and keeps the
# palm 3 mm clear of it.
GRASP_DEPTH = 0.005
SIDE = "left"


def log(robot, message):
    print(f"[{robot.time:6.2f}s] {message}", flush=True)


def plan_approach(robot, grasp):
    """The square part can be taken at four yaws about the vertical. For each,
    plan the approach - the hand raised short of the bench, a straight line
    over the part, a straight line down onto it - and use the feasible one
    whose path stays furthest from the joint limits. Returns (rotation, arm
    joints for the first point)."""
    above = grasp + [0, 0, APPROACH]
    clear = above - [BACKOFF, 0, 0]
    options = []
    for degrees in (0, 90, 180, -90):
        rotation = top_down(math.radians(degrees))
        start, ok = robot.solve_ik(SIDE, clear, rotation)
        if not ok:
            continue
        over = robot.plan_line(SIDE, clear, above, rotation, list(start.values()))
        down = over and robot.plan_line(SIDE, above, grasp, rotation, list(over[-1].values()))
        if down:
            margin = min(robot.limit_margin(q) for q in [start] + over + down)
            options.append((margin, degrees, rotation, start))
    if not options:
        raise RuntimeError(f"no grasp orientation reaches {np.round(grasp, 3)}")
    margin, degrees, rotation, start = max(options, key=lambda option: option[0])
    log(robot, f"grasping at yaw {degrees} deg (joint-limit margin {margin:.0%})")
    return rotation, start


def pick_and_place(robot, callback=None):
    """Run the scenario. Returns a dict of what happened, for check_model.py."""
    part = robot.data.body("workpiece")
    half_height = robot.model.geom("workpiece").size[2]

    robot.place_base(*START)
    robot.step(0.3, callback)
    log(robot, f"driving to the dock from {START}")
    robot.set_posture("head", "look_down")
    robot.drive_to(*DOCK, callback=callback)
    robot.step(0.5, callback)
    log(robot, f"docked at {np.round(robot.base_pose(), 4)}")

    start = part.xpos.copy()
    grasp = start + [0, 0, half_height - GRASP_DEPTH]
    rotation, clear = plan_approach(robot, grasp)
    back = np.array([-BACKOFF, 0.0, 0.0])
    robot.set_gripper(SIDE, 0.095)
    robot.move_joints(clear, 2.5, callback)
    robot.move_tcp(SIDE, grasp + [0, 0, APPROACH], rotation, 1.5, callback=callback)
    robot.move_tcp(SIDE, grasp, rotation, 1.5, callback=callback)
    robot.step(0.3, callback)
    robot.set_gripper(SIDE, 0.0)
    robot.step(0.8, callback)
    gap = robot.gripper_gap(SIDE)
    log(robot, f"gripped, jaw gap {gap * 1000:.1f} mm")

    def in_hand():  # the part's position in the grasp frame
        position, rot = robot.tcp_pose(SIDE)
        return rot.T @ (part.xpos - position)
    held = in_hand()

    robot.move_tcp(SIDE, grasp + [0, 0, APPROACH], rotation, 1.5, callback=callback)
    robot.step(0.3, callback)
    lifted = part.xpos[2] - start[2]
    log(robot, f"lifted the part {lifted * 1000:.0f} mm")

    robot.drive_to(DOCK[0], DOCK[1] + STRAFE, DOCK[2], speed=0.2, callback=callback)
    robot.step(0.5, callback)
    slip = np.linalg.norm(in_hand() - held)
    log(robot, f"strafed {STRAFE} m, the part slipped {slip * 1000:.1f} mm in the hand")

    target = grasp + [0, STRAFE, 0]
    robot.move_tcp(SIDE, target, rotation, 1.5, callback=callback)
    robot.set_gripper(SIDE, 0.095)
    robot.step(0.8, callback)
    robot.move_tcp(SIDE, target + [0, 0, APPROACH], rotation, 1.0, callback=callback)
    robot.move_tcp(SIDE, target + [0, 0, APPROACH] + back, rotation, 1.5, callback=callback)
    robot.set_posture("head", "home")
    robot.move_joints({j: 0.0 for j in robot.arm_joints(SIDE)}, 2.5, callback)
    robot.step(1.0, callback)
    placed = part.xpos.copy()
    log(robot, f"placed at {np.round(placed, 4)}, moved {np.round(placed - start, 4)}")
    return dict(gap=gap, lifted=lifted, slip=slip, start=start, placed=placed,
                expected=start + [0, STRAFE, 0])


class Recorder:
    """Pipe frames straight into ffmpeg as H.264 (see CLAUDE.md: no mp4v)."""

    def __init__(self, robot, path, camera, fps=30, size=(1280, 720)):
        if not shutil.which("ffmpeg"):
            raise SystemExit("--record needs ffmpeg on PATH")
        self.robot, self.camera, self.size = robot, camera, size
        self.period, self.next = 1.0 / fps, 0.0
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.process = subprocess.Popen(
            ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", f"{size[0]}x{size[1]}", "-r", str(fps), "-i", "-",
             "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)],
            stdin=subprocess.PIPE)

    def __call__(self, robot):
        if robot.time >= self.next:
            self.next += self.period
            self.process.stdin.write(robot.render(self.camera, size=self.size).tobytes())

    def close(self):
        self.process.stdin.close()
        self.process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--headless", action="store_true", help="no window, just run")
    mode.add_argument("--record", type=Path, metavar="MP4", help="render to an H.264 video")
    parser.add_argument("--camera", default="overview",
                        help="camera to record (overview, chase, head_d435, ...)")
    args = parser.parse_args()

    robot = Franzi()
    if args.headless:
        pick_and_place(robot)
    elif args.record:
        recorder = Recorder(robot, args.record, args.camera)
        try:
            pick_and_place(robot, recorder)
        finally:
            recorder.close()
            robot.close()
        print(f"wrote {args.record}")
    else:
        import mujoco.viewer

        with mujoco.viewer.launch_passive(robot.model, robot.data) as viewer:
            wall = time.monotonic() - robot.time

            def sync(r):
                # Draw every 8th physics step (~60 Hz) and hold the
                # simulation back to wall-clock time.
                if round(r.time / r.model.opt.timestep) % 8:
                    return
                if not viewer.is_running():
                    sys.exit(0)
                viewer.sync()
                time.sleep(max(0.0, r.time - (time.monotonic() - wall)))

            pick_and_place(robot, sync)
            while viewer.is_running():  # keep the window up; physics idles on
                robot.step(1 / 60)
                viewer.sync()
                time.sleep(1 / 60)


if __name__ == "__main__":
    main()
