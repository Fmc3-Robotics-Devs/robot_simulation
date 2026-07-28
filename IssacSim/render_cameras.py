#!/usr/bin/env python3
"""Render the robot's cameras in Isaac and write RGB + depth to disk.

This is the capability the RViz/MoveIt stack cannot provide at all: it carries
the camera frames in TF but produces no images, which is why hand-eye
calibration there can only ever be simulated geometrically and why depth-based
grasping cannot be developed there.

The cameras are spawned as children of the URDF's camera links, so their
extrinsics are the URDF's nominal values - the same numbers the ROS side
plans against. Intrinsics are the D435 colour stream's nominal 69 degree
horizontal field of view.

    conda run -n env_isaaclab python -u IssacSim/render_cameras.py

Note the -u: Isaac exits the process abruptly enough to lose buffered stdout.
"""

import argparse
import os
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
USD = REPO / "IssacSim" / "usd" / "franzi.usd"

# Nominal D435 colour intrinsics: 69 deg horizontal FOV.
# horizontal_aperture / focal_length = 2 * tan(hfov / 2)
D435 = dict(focal_length=1.93, horizontal_aperture=2.652, clipping_range=(0.05, 20.0))
# D405 is a close-range camera: much shorter working distance.
D405 = dict(focal_length=1.88, horizontal_aperture=2.782, clipping_range=(0.02, 1.0))

CAMERAS = {
    "head_d435": ("head_d435_Link", D435),
    "body_d435": ("body_d435_Link", D435),
    "left_wrist_d405": ("left_wrist_d405_Link", D405),
    "right_wrist_d405": ("right_wrist_d405_Link", D405),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default=str(REPO / "IssacSim" / "renders"))
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--headless", action="store_true", default=True)
    args = parser.parse_args()

    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=args.headless, enable_cameras=True)
    simulation_app = launcher.app

    import numpy as np
    import isaaclab.sim as sim_utils
    from isaaclab.sensors import Camera, CameraCfg

    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=1.0 / 60.0, device="cuda:0")
    )

    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=2500.0).func(
        "/World/light", sim_utils.DomeLightCfg(intensity=2500.0)
    )
    sim_utils.UsdFileCfg(usd_path=str(USD)).func(
        "/World/Robot", sim_utils.UsdFileCfg(usd_path=str(USD))
    )

    # A bench in front of the robot, so the cameras have something to look at
    # other than an empty floor.
    bench = sim_utils.CuboidCfg(
        size=(0.5, 0.7, 0.03),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.72, 0.60, 0.44)),
    )
    bench.func("/World/bench", bench, translation=(0.73, 0.16, 0.785))
    block = sim_utils.CuboidCfg(
        size=(0.05, 0.05, 0.09),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.90, 0.45, 0.15)),
    )
    block.func("/World/block", block, translation=(0.44, 0.16, 0.845))

    cameras = {}
    for name, (link, intrinsics) in CAMERAS.items():
        cameras[name] = Camera(
            CameraCfg(
                prim_path=f"/World/Robot/{link}/{name}",
                update_period=0.0,
                height=args.height,
                width=args.width,
                data_types=["rgb", "distance_to_image_plane"],
                spawn=sim_utils.PinholeCameraCfg(**intrinsics),
            )
        )

    sim.reset()
    # Rendering is deferred: the first frames come back empty until the
    # replicator pipeline has been pumped.
    for _ in range(12):
        sim.step()
        for camera in cameras.values():
            camera.update(sim.get_physics_dt())

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    for name, camera in cameras.items():
        rgb = camera.data.output["rgb"][0].cpu().numpy()
        depth = camera.data.output["distance_to_image_plane"][0].cpu().numpy()
        np.save(output / f"{name}_depth.npy", depth)

        finite = depth[np.isfinite(depth)]
        print(
            f"{name:18s} rgb {rgb.shape} mean {rgb[..., :3].mean():6.1f}   "
            f"depth {depth.shape} "
            f"{finite.min():.3f}-{finite.max():.3f} m"
            if finite.size
            else f"{name:18s} rgb {rgb.shape} depth all invalid"
        )

        try:
            from PIL import Image

            Image.fromarray(rgb[..., :3].astype("uint8")).save(output / f"{name}_rgb.png")
        except ImportError:
            np.save(output / f"{name}_rgb.npy", rgb)

    print(f"wrote {len(cameras)} camera renders to {output}", flush=True)

    # simulation_app.close() does not reliably return with the RTX pipeline
    # running, so the process is handed back to the OS instead of waiting.
    simulation_app.close()
    os._exit(0)


if __name__ == "__main__":
    main()
