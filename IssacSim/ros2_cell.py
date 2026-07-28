#!/usr/bin/env python3
"""Run the cell in Isaac and publish it on ROS 2.

Isaac takes over the job the demo driver does today: it publishes the clock,
the joint states and the chassis transform, so `robot_state_publisher`, MoveIt
and the pick-and-place task carry on unchanged - except that now there are real
camera images alongside them.

Topic layout follows the RealSense driver's, one namespace per camera:

    /head_d435/color/image_raw          /head_d435/color/camera_info
    /head_d435/depth/image_rect_raw     /head_d435/depth/camera_info

so that migrating to hardware is a matter of not starting this and starting the
RealSense driver instead. Nothing downstream needs to know which one it is.

    IssacSim/run_ros2_cell.sh            # sets up the ROS 2 libraries first

Frame ids are the URDF link names. Note that those are not optical frames in
the RealSense sense - the real driver publishes `*_color_optical_frame`, which
the URDF does not currently contain. That has to be reconciled before hardware.

STATUS: cameras, clock, joint states and the chassis transform are verified
publishing. The two lidars are not. They are created at the right prims now -
the config name is case-sensitive and takes the USD asset's spelling, and the
command returns the OmniLidar nested inside the referenced asset rather than
the path it was handed - but ROS2RtxLidarHelper still reports "Render product
not attached to RTX Lidar (Camera or OmniLidar prims are required)". The next
thing to check is the type of the prim the command hands back: the helper wants
a Camera or OmniLidar and is evidently getting something else.
"""

import argparse
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

# The SRDF's virtual joint hangs the robot off `moveit_root`, not `base_link`.
# Publishing odom -> base_link instead would give base_link two parents and
# break the tree.
ODOM_FRAME = "odom"
ROBOT_ROOT_FRAME = "moveit_root"

# The warehouse gives SLAM something to close a loop on. The simple shell is
# used rather than the fully stocked one: its racking sits where the cell does.
WAREHOUSE_USD = "Isaac/Environments/Simple_Warehouse/warehouse.usd"

# Config names come from the USD asset stems in SUPPORTED_LIDAR_CONFIGS, and
# the match is case-sensitive. The JSON files under isaacsim.sensors.rtx/data
# are spelled differently - SICK_tim781.json against the asset's SICK_TIM781 -
# and using the JSON spelling fails with "config not found", after which the
# sensor prim never exists and the render product reports the far less helpful
# "No valid sensor paths provided".
#
# No Livox profile ships with Isaac, so the MID360 borrows a comparable 32-beam
# spinner. The 2D unit is what drives Nav2, so its profile matters more: the
# TIM781 is a real 2D safety scanner and behaves like one.
LIDARS = {
    "scan_2d": (
        "lidar_2Dlidar_Link",
        "SICK_TIM781",
        "laser_scan",
        "scan",
    ),
    "mid360": (
        "MID360_Link",
        "HESAI_XT32_SD10",
        "point_cloud",
        "mid360/points",
    ),
}


def build_camera_graph(camera_name, camera_prim, width, height, frame_id):
    """Wire one camera into ROS 2: colour, depth and both camera_infos."""
    import omni.graph.core as og
    from isaacsim.core.utils.prims import set_targets

    graph_path = f"/ROS2/{camera_name}"
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("Colour", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("Depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("Info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "Colour.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "Depth.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "Info.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "Colour.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "Depth.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "Info.inputs:renderProductPath"),
                ("Context.outputs:context", "Colour.inputs:context"),
                ("Context.outputs:context", "Depth.inputs:context"),
                ("Context.outputs:context", "Info.inputs:context"),
            ],
            keys.SET_VALUES: [
                ("RenderProduct.inputs:width", width),
                ("RenderProduct.inputs:height", height),
                ("Colour.inputs:type", "rgb"),
                ("Colour.inputs:topicName", f"{camera_name}/color/image_raw"),
                ("Colour.inputs:frameId", frame_id),
                ("Depth.inputs:type", "depth"),
                ("Depth.inputs:topicName", f"{camera_name}/depth/image_rect_raw"),
                ("Depth.inputs:frameId", frame_id),
                ("Info.inputs:topicName", f"{camera_name}/color/camera_info"),
                ("Info.inputs:frameId", frame_id),
            ],
        },
    )
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    set_targets(
        prim=stage.GetPrimAtPath(f"{graph_path}/RenderProduct"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[camera_prim],
    )


def build_lidar_graph(name, lidar_prim, publish_type, topic, frame_id):
    """Publish one RTX lidar. Same render-product pattern as the cameras."""
    import omni.graph.core as og
    import omni.usd
    from isaacsim.core.utils.prims import set_targets

    graph_path = f"/ROS2/{name}"
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("Publish", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "Publish.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "Publish.inputs:renderProductPath"),
                ("Context.outputs:context", "Publish.inputs:context"),
            ],
            keys.SET_VALUES: [
                ("Publish.inputs:type", publish_type),
                ("Publish.inputs:topicName", topic),
                ("Publish.inputs:frameId", frame_id),
            ],
        },
    )
    stage = omni.usd.get_context().get_stage()
    set_targets(
        prim=stage.GetPrimAtPath(f"{graph_path}/RenderProduct"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[lidar_prim],
    )


def build_robot_graph(robot_prim):
    """Clock, joint states and the chassis transform - the demo driver's job."""
    import omni.graph.core as og
    import omni.usd
    from isaacsim.core.utils.prims import set_targets

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/ROS2/robot", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("JointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("Odom", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "Clock.inputs:execIn"),
                ("Tick.outputs:tick", "JointState.inputs:execIn"),
                ("Tick.outputs:tick", "Odom.inputs:execIn"),
                ("Context.outputs:context", "Clock.inputs:context"),
                ("Context.outputs:context", "JointState.inputs:context"),
                ("Context.outputs:context", "Odom.inputs:context"),
                ("SimTime.outputs:simulationTime", "Clock.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "JointState.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "Odom.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("Clock.inputs:topicName", "clock"),
                ("JointState.inputs:topicName", "joint_states"),
                ("Odom.inputs:topicName", "tf"),
                ("Odom.inputs:parentFrameId", ODOM_FRAME),
                ("Odom.inputs:childFrameId", ROBOT_ROOT_FRAME),
            ],
        },
    )
    stage = omni.usd.get_context().get_stage()
    set_targets(
        prim=stage.GetPrimAtPath("/ROS2/robot/JointState"),
        attribute="inputs:targetPrim",
        target_prim_paths=[robot_prim],
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true", help="Open the Isaac window.")
    parser.add_argument("--dock", default="feeder", choices=STATIONS)
    parser.add_argument(
        "--no-warehouse", action="store_true", help="Bare ground plane instead."
    )
    args = parser.parse_args()

    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=not args.gui, enable_cameras=True)
    simulation_app = launcher.app

    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    import omni.usd
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import Articulation, ArticulationCfg

    import apriltags
    from render_cell import spawn_textured_quad

    cell = Cell.load()
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=1.0 / 60.0, device="cuda:0", gravity=(0.0, 0.0, 0.0))
    )

    # The floor has to land on the wheel contact plane, not on z = 0: the world
    # frame here is the ROS odom frame, whose origin is base_link.
    floor_z = cell["ground_z"]
    if args.no_warehouse:
        ground = sim_utils.GroundPlaneCfg()
        ground.func("/World/ground", ground, translation=(0.0, 0.0, floor_z))
    else:
        from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

        warehouse = sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/{WAREHOUSE_USD.split('Isaac/', 1)[1]}"
        )
        warehouse.func("/World/warehouse", warehouse, translation=(0.0, 0.0, floor_z))
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
        spawn_textured_quad(
            stage,
            f"/World/tag_{station}",
            cell["tag.size"] * apriltags.sheet_scale(),
            texture,
            cell.tag_position(station),
        )

    workpiece = sim_utils.CuboidCfg(size=tuple(cell["workpiece.size"]), visual_material=orange)
    workpiece.func("/World/workpiece", workpiece, translation=cell.part_position("feeder"))

    dock_x, dock_y, _ = cell.dock_pose(args.dock)
    robot = Articulation(
        ArticulationCfg(
            prim_path="/World/Robot",
            spawn=sim_utils.UsdFileCfg(usd_path=str(USD)),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(dock_x, dock_y, 0.0),
                joint_pos={**WORK_POSTURE, **LOOK_DOWN},
            ),
            actuators={
                "all": ImplicitActuatorCfg(
                    joint_names_expr=[".*"], stiffness=0.0, damping=0.0
                )
            },
        )
    )

    # Cameras are plain USD cameras here rather than IsaacLab sensors: the
    # bridge renders them itself through a render product, and a second
    # annotator on the same prim only costs frame time.
    from pxr import UsdGeom

    for name, (link, intrinsics, _) in CAMERAS.items():
        prim_path = f"/World/Robot/{link}/{name}"
        camera = UsdGeom.Camera.Define(stage, prim_path)
        camera.CreateFocalLengthAttr(intrinsics["focal_length"])
        camera.CreateHorizontalApertureAttr(intrinsics["horizontal_aperture"])
        camera.CreateVerticalApertureAttr(
            intrinsics["horizontal_aperture"] * CAMERAS[name][2][1] / CAMERAS[name][2][0]
        )
        near, far = intrinsics["clipping_range"]
        camera.CreateClippingRangeAttr((near, far))

    # RTX lidars are created through the sensor command, not a spawner config.
    import omni.kit.commands

    for name, (link, config, _, _) in LIDARS.items():
        # path is a leaf name; passing a full path flattens its slashes into
        # underscores and the sensor lands at the stage root.
        ok, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=name,
            parent=f"/World/Robot/{link}",
            config=config,
            translation=(0.0, 0.0, 0.0),
        )
        print(f"lidar {name}: {'created ' + str(sensor.GetPath()) if ok else 'FAILED'}",
              flush=True)

    sim.reset()
    root = robot.data.default_root_state.clone()
    robot.write_root_pose_to_sim(root[:, :7])
    robot.write_root_velocity_to_sim(root[:, 7:])
    robot.write_joint_state_to_sim(
        robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
    )
    robot.reset()

    build_robot_graph("/World/Robot")
    for name, (link, _, (width, height)) in CAMERAS.items():
        build_camera_graph(name, f"/World/Robot/{link}/{name}", width, height, link)
    for name, (link, _, publish_type, topic) in LIDARS.items():
        build_lidar_graph(
            name, f"/World/Robot/{link}/{name}", publish_type, topic, link
        )

    print("publishing:", flush=True)
    print(f"  /clock  /joint_states  tf {ODOM_FRAME} -> {ROBOT_ROOT_FRAME}", flush=True)
    for name in CAMERAS:
        print(
            f"  /{name}/color/image_raw  /{name}/color/camera_info  "
            f"/{name}/depth/image_rect_raw",
            flush=True,
        )
    for name, (_, config, publish_type, topic) in LIDARS.items():
        print(f"  /{topic}  ({publish_type}, {config})", flush=True)
    print("\nrunning - Ctrl-C to stop", flush=True)

    try:
        while simulation_app.is_running():
            sim.step()
    except KeyboardInterrupt:
        pass
    os._exit(0)


if __name__ == "__main__":
    main()
