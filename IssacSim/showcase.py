#!/usr/bin/env python3
"""Photograph and film the painted Franzi robot with an external camera.

The cell is the same one `ros2_cell.py` publishes - same task.yaml layout,
same warehouse, same work posture at the feeder dock - but the camera is a
free-standing one pointed *at* the robot, which none of the onboard cameras
can do. Stills go to `renders/`, orbit frames to a subdirectory for ffmpeg
to assemble (H.264, per repo convention).

    conda run -n env_isaaclab python -u IssacSim/showcase.py --test   # stills only
    conda run -n env_isaaclab python -u IssacSim/showcase.py          # stills + orbit
"""

import argparse
import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cell import LOOK_DOWN, STATIONS, WORK_POSTURE, Cell  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
USD = REPO / "IssacSim" / "usd" / "franzi.usd"
TEXTURES = REPO / "IssacSim" / "usd" / "textures"
WAREHOUSE_USD = "Isaac/Environments/Simple_Warehouse/warehouse.usd"

# (eye, target) in the docked robot's frame: x forward, y left, z up from the
# wheel plane. Chosen for a ~1.6 m robot with a 16 mm-equivalent lens.
# (eye, target) in the docked robot's frame. The default dock is `home`, the
# odom origin, where robot frame == world frame. Portrait eyes stay short of
# x = 2.0: the machine's front face is at x = 2.18 and its door faces -x, so
# it is shot from the aisle, never from behind.
VIEWS = {
    "front": ((1.9, -1.2, 1.45), (0.0, 0.0, 0.85)),
    "three_quarter": ((1.9, 1.4, 1.6), (0.0, 0.0, 0.85)),
    "side": ((0.2, -3.0, 1.3), (0.0, 0.0, 0.85)),
    "chest": ((1.15, -0.3, 1.3), (0.0, 0.0, 1.15)),
    "machine": ((0.0, 1.7, 1.7), (2.0, 0.3, 0.9)),
    "stock": ((0.8, -2.6, 1.7), (2.2, -4.0, 0.82)),
    "cell": ((-4.6, 0.0, 2.4), (1.6, 0.0, 0.9)),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dock", default="home", choices=list(STATIONS) + ["home"])
    parser.add_argument("--output", default=str(REPO / "IssacSim" / "renders"))
    parser.add_argument("--frames", type=int, default=240, help="Orbit frame count.")
    # Orbit centre sits 0.6 m behind the robot so the circle's far side stays
    # out of the machine's front face at x = 2.18.
    parser.add_argument("--radius", type=float, default=2.4)
    parser.add_argument("--test", action="store_true", help="Stills only, 720p.")
    parser.add_argument("--no-warehouse", action="store_true")
    args = parser.parse_args()
    width, height = (1280, 720) if args.test else (1920, 1080)

    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=True, enable_cameras=True)
    simulation_app = launcher.app

    import numpy as np
    import torch
    import omni.usd
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import Articulation, ArticulationCfg
    from isaaclab.sensors import Camera, CameraCfg
    from PIL import Image

    import paint
    import scenery

    cell = Cell.load()
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=1.0 / 60.0, device="cuda:0", gravity=(0.0, 0.0, 0.0))
    )

    stage = omni.usd.get_context().get_stage()
    warehouse_path = None
    if not args.no_warehouse:
        from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

        warehouse_path = f"{ISAAC_NUCLEUS_DIR}/{WAREHOUSE_USD.split('Isaac/', 1)[1]}"
    scenery.build(stage, sim_utils, cell, TEXTURES, warehouse_path)

    dock_x, dock_y, dock_yaw = cell.dock_pose(args.dock)
    robot = Articulation(
        ArticulationCfg(
            prim_path="/World/Robot",
            spawn=sim_utils.UsdFileCfg(usd_path=str(USD)),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(dock_x, dock_y, 0.0),
                rot=(math.cos(dock_yaw / 2), 0.0, 0.0, math.sin(dock_yaw / 2)),
                joint_pos={**WORK_POSTURE, **LOOK_DOWN},
            ),
            actuators={
                "all": ImplicitActuatorCfg(
                    joint_names_expr=[".*"], stiffness=0.0, damping=0.0
                )
            },
        )
    )
    paint.paint_robot(stage)
    paint.spawn_logo(stage)

    camera = Camera(
        CameraCfg(
            prim_path="/World/viewcam",
            update_period=0.0,
            width=width,
            height=height,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=16.0, clipping_range=(0.05, 60.0)
            ),
        )
    )

    sim.reset()
    root = robot.data.default_root_state.clone()
    robot.write_root_pose_to_sim(root[:, :7])
    robot.write_root_velocity_to_sim(root[:, 7:])
    robot.write_joint_state_to_sim(
        robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
    )
    robot.reset()

    cos_yaw, sin_yaw = math.cos(dock_yaw), math.sin(dock_yaw)

    def place(eye, target):
        def world(point):
            return (
                dock_x + cos_yaw * point[0] - sin_yaw * point[1],
                dock_y + sin_yaw * point[0] + cos_yaw * point[1],
                point[2],
            )

        camera.set_world_poses_from_view(
            torch.tensor([world(eye)], device=sim.device),
            torch.tensor([world(target)], device=sim.device),
        )

    def shoot(settle):
        for _ in range(settle):
            sim.step()
            camera.update(sim.get_physics_dt())
        return camera.data.output["rgb"][0].cpu().numpy()[..., :3].astype(np.uint8)

    place(*VIEWS["front"])
    shoot(24)  # let RTX warm up before the first real frame

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    for name, (eye, target) in VIEWS.items():
        place(eye, target)
        Image.fromarray(shoot(12)).save(output / f"franzi_painted_{name}.png")
        print(f"still: {output / f'franzi_painted_{name}.png'}", flush=True)

    if not args.test:
        frames_dir = output / "orbit_frames"
        frames_dir.mkdir(exist_ok=True)
        for index in range(args.frames):
            angle = 2.0 * math.pi * index / args.frames
            eye = (
                -0.6 + args.radius * math.cos(angle),
                args.radius * math.sin(angle),
                1.5,
            )
            place(eye, (0.0, 0.0, 0.85))
            Image.fromarray(shoot(2)).save(frames_dir / f"frame_{index:04d}.png")
            if index % 30 == 0:
                print(f"orbit frame {index}/{args.frames}", flush=True)
        print(f"orbit frames in {frames_dir}", flush=True)

    # close() does not reliably return with the RTX pipeline up.
    os._exit(0)


if __name__ == "__main__":
    main()
