#!/usr/bin/env python3
"""Build the machine-tending cell in Isaac and look at it through the cameras.

The layout is read from the ROS side's `task.yaml`, and the Isaac world frame is
the ROS `odom` frame - same origin, same axes - so a pose that works in one
works in the other. The benches, the workpiece and the tags are the same
numbers the MoveIt planning scene uses.

What is genuinely new here, and impossible in the RViz stack, is that the
AprilTags are real textures and the detector is real OpenCV: the perception
input is measured from a rendered image rather than computed from ground truth.

    conda run -n env_isaaclab python -u IssacSim/render_cell.py          # render
    conda run -n env_isaaclab python -u IssacSim/render_cell.py --gui    # and watch

Note the -u: Isaac exits abruptly enough to lose buffered stdout.
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

D435 = dict(focal_length=1.93, horizontal_aperture=2.652, clipping_range=(0.05, 20.0))
D405 = dict(focal_length=1.88, horizontal_aperture=2.782, clipping_range=(0.02, 1.0))
CAMERAS = {
    "head_d435": ("head_d435_Link", D435, (1280, 720)),
    "body_d435": ("body_d435_Link", D435, (640, 480)),
    "left_wrist_d405": ("left_wrist_d405_Link", D405, (640, 480)),
    "right_wrist_d405": ("right_wrist_d405_Link", D405, (640, 480)),
}


def spawn_textured_quad(stage, path, size, texture, position):
    """A flat, matte, textured square lying in the XY plane, facing +Z.

    Built with plain USD rather than an IsaacLab material config, which cannot
    take a texture path. UV (0,0) maps to the lower-left of the image, so the
    marker comes out unmirrored when viewed from above - which matters, because
    a mirrored AprilTag does not decode.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    half = size / 2.0
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(
        [
            Gf.Vec3f(-half, -half, 0.0),
            Gf.Vec3f(half, -half, 0.0),
            Gf.Vec3f(half, half, 0.0),
            Gf.Vec3f(-half, half, 0.0),
        ]
    )
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0.0, 0.0, 1.0)] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    mesh.CreateExtentAttr([Gf.Vec3f(-half, -half, 0.0), Gf.Vec3f(half, half, 0.0)])

    coords = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    )
    coords.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    UsdGeom.Xformable(mesh).AddTranslateOp().Set(Gf.Vec3d(*position))

    material = UsdShade.Material.Define(stage, f"{path}/Material")
    surface = UsdShade.Shader.Define(stage, f"{path}/Material/Surface")
    surface.CreateIdAttr("UsdPreviewSurface")
    # Matte: a specular highlight across a marker blows out its cells.
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    surface.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

    reader = UsdShade.Shader.Define(stage, f"{path}/Material/Reader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    diffuse = UsdShade.Shader.Define(stage, f"{path}/Material/Diffuse")
    diffuse.CreateIdAttr("UsdUVTexture")
    diffuse.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(texture))
    diffuse.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result"
    )
    diffuse.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        diffuse.ConnectableAPI(), "rgb"
    )
    material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(mesh).Apply(mesh.GetPrim())
    UsdShade.MaterialBindingAPI(mesh).Bind(material)
    return mesh


def usd_world_pose(stage, prim_path):
    """World pose of a prim, straight from USD.

    IsaacLab's Camera.data.pos_w reports the origin for a camera parented under
    an articulation link, so the transform is read from the stage instead.
    """
    import numpy as np
    from pxr import Usd, UsdGeom

    matrix = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    rows = np.array([[matrix[r][c] for c in range(4)] for r in range(4)])
    # USD matrices are row-vector convention: the basis vectors are the rows.
    return rows[3, :3], rows[:3, :3].T


def usd_local_pose(stage, prim_path):
    """A prim's transform relative to its parent, from USD.

    Safe to read from the stage because it is authored and static - unlike
    anything driven by physics, which only exists in Fabric at runtime.
    """
    import numpy as np
    from pxr import Usd, UsdGeom

    matrix = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path)).GetLocalTransformation(
        Usd.TimeCode.Default()
    )
    rows = np.array([[matrix[r][c] for c in range(4)] for r in range(4)])
    return rows[3, :3], rows[:3, :3].T


def link_world_pose(robot, link_name):
    """A link's pose as physics actually has it.

    The USD stage keeps the authored pose; PhysX publishes the simulated one to
    Fabric. Reading link poses off the stage reports the robot in its import
    posture no matter what the joints are doing.
    """
    index = robot.body_names.index(link_name)
    return (
        robot.data.body_pos_w[0, index].cpu().numpy(),
        quaternion_matrix(robot.data.body_quat_w[0, index].cpu().numpy()),
    )


def quaternion_matrix(quaternion):
    """Rotation matrix from a (w, x, y, z) quaternion."""
    import numpy as np

    w, x, y, z = quaternion
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true", help="Open the Isaac window.")
    parser.add_argument("--dock", default="feeder", choices=STATIONS)
    parser.add_argument("--output", default=str(REPO / "IssacSim" / "renders"))
    args = parser.parse_args()

    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=not args.gui, enable_cameras=True)
    simulation_app = launcher.app

    import numpy as np
    import omni.usd
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import Articulation, ArticulationCfg
    from isaaclab.sensors import Camera, CameraCfg

    import apriltags

    cell = Cell.load()
    print(f"cell loaded from {Cell.load.__doc__ and 'task.yaml'}", flush=True)

    # No physics is wanted here: the robot should stand exactly where it is put
    # so the rendered geometry matches the poses the ROS side plans against.
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=1.0 / 60.0, device="cuda:0", gravity=(0.0, 0.0, 0.0))
    )

    ground = sim_utils.GroundPlaneCfg()
    ground.func("/World/ground", ground, translation=(0.0, 0.0, cell["ground_z"]))
    dome = sim_utils.DomeLightCfg(intensity=1200.0, color=(0.95, 0.95, 1.0))
    dome.func("/World/dome", dome)
    key = sim_utils.DistantLightCfg(intensity=2500.0, angle=1.0)
    key.func("/World/key", key, orientation=(0.86, 0.35, 0.35, 0.0))

    wood = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.62, 0.50, 0.36), roughness=0.8)
    steel = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.45, 0.48, 0.55), roughness=0.5)
    orange = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.90, 0.45, 0.15), roughness=0.6)

    stage = omni.usd.get_context().get_stage()
    for station in STATIONS:
        slab = sim_utils.CuboidCfg(size=cell.slab_size, visual_material=wood)
        slab.func(f"/World/bench_{station}/slab", slab, translation=cell.slab_position(station))
        for index, position in enumerate(cell.leg_positions(station)):
            leg = sim_utils.CuboidCfg(size=cell.leg_size, visual_material=steel)
            leg.func(f"/World/bench_{station}/leg_{index}", leg, translation=position)

        tag_id = cell.tag_id(station)
        texture = apriltags.write_marker(tag_id, TEXTURES / f"tag_{tag_id}.png")
        # The sheet is wider than the marker by its quiet zone; the marker
        # itself has to end up exactly `tag.size` across or every pose estimate
        # scales with the error.
        spawn_textured_quad(
            stage,
            f"/World/tag_{station}",
            cell["tag.size"] * apriltags.sheet_scale(),
            texture,
            cell.tag_position(station),
        )

    workpiece = sim_utils.CuboidCfg(size=tuple(cell["workpiece.size"]), visual_material=orange)
    workpiece.func("/World/workpiece", workpiece, translation=cell.part_position("feeder"))

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
            # Zero gains on purpose. The converter bakes position drives into
            # the USD whose targets default to zero, so any non-zero stiffness
            # drags the robot back to all-joints-zero within a few steps - the
            # posture is written, held for an instant, and quietly undone.
            # Nothing is being controlled here, so nothing should pull.
            actuators={
                "all": ImplicitActuatorCfg(
                    joint_names_expr=[".*"], stiffness=0.0, damping=0.0
                )
            },
        )
    )

    cameras = {}
    for name, (link, intrinsics, (width, height)) in CAMERAS.items():
        cameras[name] = Camera(
            CameraCfg(
                prim_path=f"/World/Robot/{link}/{name}",
                update_period=0.0,
                width=width,
                height=height,
                data_types=["rgb", "distance_to_image_plane"],
                spawn=sim_utils.PinholeCameraCfg(**intrinsics),
            )
        )

    sim.reset()
    # Configuring init_state is not enough: the articulation's default state has
    # to be written into the simulation explicitly, or the robot stays at the
    # origin and every camera looks at the wrong part of the world.
    root = robot.data.default_root_state.clone()
    robot.write_root_pose_to_sim(root[:, :7])
    robot.write_root_velocity_to_sim(root[:, 7:])
    robot.write_joint_state_to_sim(
        robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
    )
    robot.reset()

    for _ in range(16):
        sim.step()
        for camera in cameras.values():
            camera.update(sim.get_physics_dt())

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    try:
        from PIL import Image
    except ImportError:
        Image = None

    for name, camera in cameras.items():
        rgb = camera.data.output["rgb"][0].cpu().numpy()[..., :3]
        depth = camera.data.output["distance_to_image_plane"][0].cpu().numpy()
        np.save(output / f"{name}_depth.npy", depth)
        if Image is not None:
            Image.fromarray(rgb.astype("uint8")).save(output / f"{name}_rgb.png")

    # -- the part that could not be done without a renderer ----------------
    head = cameras["head_d435"]
    rgb = head.data.output["rgb"][0].cpu().numpy()[..., :3].astype(np.uint8)
    intrinsics = head.data.intrinsic_matrices[0].cpu().numpy()
    detections = apriltags.detect(rgb, intrinsics, cell["tag.size"])

    # Link pose from physics, camera offset from USD: the only combination that
    # is right for both. IsaacLab's Camera.data.pos_w reports the origin here.
    link_position, link_rotation = link_world_pose(robot, "head_d435_Link")
    local_position, local_rotation = usd_local_pose(
        stage, "/World/Robot/head_d435_Link/head_d435"
    )
    where = link_position + link_rotation @ local_position
    orientation = link_rotation @ local_rotation

    stale, _ = usd_world_pose(stage, "/World/Robot/head_d435_Link/head_d435")
    print(
        f"\nrobot docked at {cell.dock_pose(args.dock)[:2]}\n"
        f"head camera: physics ({where[0]:+.3f}, {where[1]:+.3f}, {where[2]:+.3f})  "
        f"stage ({stale[0]:+.3f}, {stale[1]:+.3f}, {stale[2]:+.3f})",
        flush=True,
    )
    posed = {
        name: round(float(robot.data.joint_pos[0][robot.joint_names.index(name)]), 4)
        for name in list(WORK_POSTURE) + list(LOOK_DOWN)
    }
    print(f"posture in sim: {posed}", flush=True)
    print(f"head camera {rgb.shape[1]}x{rgb.shape[0]}, "
          f"fx={intrinsics[0][0]:.1f} px", flush=True)
    if not detections:
        print("no AprilTag decoded from the rendered image", flush=True)
    # OpenCV puts +x right, +y down, +z forward; a USD camera has +y up and
    # looks down -z. Skipping this flip lands the tag behind the robot.
    to_usd = np.diag([1.0, -1.0, -1.0])
    for tag_id, (translation, _) in sorted(detections.items()):
        station = next(s for s in STATIONS if cell.tag_id(s) == tag_id)
        measured = where + orientation @ (to_usd @ translation)
        truth = np.array(cell.tag_position(station))
        print(
            f"tag {tag_id} ({station}): measured "
            f"({measured[0]:+.4f}, {measured[1]:+.4f}, {measured[2]:+.4f})  "
            f"error {np.linalg.norm(measured - truth) * 1000:6.1f} mm  "
            f"range {np.linalg.norm(translation):.3f} m",
            flush=True,
        )

    print(f"\nwrote renders to {output}", flush=True)

    if args.gui:
        print("window open - close it or press Ctrl-C to finish", flush=True)
        while simulation_app.is_running():
            sim.step()
    # close() does not reliably return with the RTX pipeline up, and anything
    # after it never runs.
    os._exit(0)


if __name__ == "__main__":
    main()
