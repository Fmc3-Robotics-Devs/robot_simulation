#!/usr/bin/env python3
"""Run the cell in Isaac as the sensor stage of the ROS 2 stack.

Two modes:

* **mirror** (default): the ROS side keeps driving the robot exactly as it does
  against RViz - MoveIt executes on the demo controllers, the base integrates
  its own pose - and Isaac follows along by subscribing to `/joint_states` and
  `/base/pose`, posing its articulation to match. What Isaac adds is what the
  RViz stack cannot make: rendered camera images of the real scene geometry,
  from which a real AprilTag detector measures the stations. Perception becomes
  measured instead of mocked while the control stack stays byte-for-byte the
  one that will move to hardware.

* **--standalone**: the original self-publishing mode - Isaac owns the clock,
  the joint states and the chassis transform - for looking at the cell without
  the ROS stack running.

Cameras are IsaacLab Camera sensors, the same construction `render_cell.py`
already validated by decoding an AprilTag from a render, and publish with
optical-frame ids (`<camera>_optical`). The transform from each URDF camera
link to its optical frame is printed at start-up; the ROS side publishes it as
a static transform (see franzi_skills' tending launch).

    IssacSim/run_ros2_cell.sh                      # mirror, head camera
    IssacSim/run_ros2_cell.sh --standalone --gui   # just look at the cell

Topic layout follows the RealSense driver's, one namespace per camera, so
hardware migration is a matter of starting the RealSense driver instead.
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

# The SRDF's virtual joint hangs the robot off `moveit_root`, not `base_link`.
ODOM_FRAME = "odom"
ROBOT_ROOT_FRAME = "moveit_root"

WAREHOUSE_USD = "Isaac/Environments/Simple_Warehouse/warehouse.usd"

# Config names come from the USD asset stems in SUPPORTED_LIDAR_CONFIGS and the
# match is case-sensitive (the JSON files under isaacsim.sensors.rtx/data are
# spelled differently and fail). No Livox profile ships with Isaac, so the
# MID360 borrows a comparable 32-beam spinner.
# (usd prim, sensor config, publish type, topic, tf frame). The USD prim is
# the URDF link renamed - prim names cannot start with a digit - but TF only
# knows the URDF spelling, and the scan must be stamped with a frame that
# exists in that tree.
LIDARS = {
    "scan_2d": ("lidar_2Dlidar_Link", "SICK_TIM781", "laser_scan", "scan", "2Dlidar_Link"),
    "mid360": ("MID360_Link", "HESAI_XT32_SD10", "point_cloud", "mid360/points", "MID360_Link"),
}


def build_camera_graph(camera_name, camera_prim, width, height, frame_id):
    """Publish one camera's colour, depth and camera_info. Stamps use system
    time: the rest of the stack runs on wall clocks, and a sim-stamped
    detection would never share a timeline with it."""
    import omni.graph.core as og
    import omni.usd
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
                ("Colour.inputs:useSystemTime", True),
                ("Depth.inputs:type", "depth"),
                ("Depth.inputs:topicName", f"{camera_name}/depth/image_rect_raw"),
                ("Depth.inputs:frameId", frame_id),
                ("Depth.inputs:useSystemTime", True),
                ("Info.inputs:topicName", f"{camera_name}/color/camera_info"),
                ("Info.inputs:frameId", frame_id),
                ("Info.inputs:useSystemTime", True),
            ],
        },
    )
    stage = omni.usd.get_context().get_stage()
    set_targets(
        prim=stage.GetPrimAtPath(f"{graph_path}/RenderProduct"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[camera_prim],
    )


def build_lidar_graph(name, lidar_prim_path, publish_type, topic, frame_id):
    """Publish one RTX lidar. ``lidar_prim_path`` must be the OmniLidar prim
    the sensor command actually returned - the asset nests it below the path
    the command was handed, and the helper refuses the outer Xform."""
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
                ("Publish.inputs:useSystemTime", True),
            ],
        },
    )
    stage = omni.usd.get_context().get_stage()
    set_targets(
        prim=stage.GetPrimAtPath(f"{graph_path}/RenderProduct"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[lidar_prim_path],
    )


def build_robot_graph(robot_prim):
    """Standalone mode only: clock, joint states and the chassis transform."""
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


def build_mirror_graph():
    """Subscribe to what the ROS stack says the robot is doing.

    Joint states arrive through the dedicated subscriber node; the base pose
    and the workpiece pose through generic message subscribers, because the
    bridge has no TF subscriber and both are plain PoseStamped precisely so
    this node can read them.
    """
    import omni.graph.core as og

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/ROS2/mirror", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("Joints", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("BasePose", "isaacsim.ros2.bridge.ROS2Subscriber"),
                ("Workpiece", "isaacsim.ros2.bridge.ROS2Subscriber"),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "Joints.inputs:execIn"),
                ("Tick.outputs:tick", "BasePose.inputs:execIn"),
                ("Tick.outputs:tick", "Workpiece.inputs:execIn"),
                ("Context.outputs:context", "Joints.inputs:context"),
                ("Context.outputs:context", "BasePose.inputs:context"),
                ("Context.outputs:context", "Workpiece.inputs:context"),
            ],
            keys.SET_VALUES: [
                ("Joints.inputs:topicName", "joint_states"),
                ("BasePose.inputs:messagePackage", "geometry_msgs"),
                ("BasePose.inputs:messageSubfolder", "msg"),
                ("BasePose.inputs:messageName", "PoseStamped"),
                ("BasePose.inputs:topicName", "base/pose"),
                ("Workpiece.inputs:messagePackage", "geometry_msgs"),
                ("Workpiece.inputs:messageSubfolder", "msg"),
                ("Workpiece.inputs:messageName", "PoseStamped"),
                ("Workpiece.inputs:topicName", "workpiece/pose"),
            ],
        },
    )


class Mirror:
    """Applies the subscribed joint states and base pose to the articulation.

    It also writes every link's physics pose back to the USD stage each frame.
    PhysX runs Fabric-only here and never updates the stage, but cameras
    parented under links render from the *stage* - without the write-back they
    film the import posture forever, 30 degrees away from where TF says the
    lens points. The observer camera gets a moving robot out of the same fix.
    """

    def __init__(self, robot):
        import omni.graph.core as og
        import omni.usd
        from isaacsim.core.prims import XFormPrim
        from pxr import UsdGeom

        self._og = og
        self._robot = robot
        self._index_of = {name: i for i, name in enumerate(robot.joint_names)}
        self._positions = robot.data.default_joint_pos.clone()
        self._velocities = robot.data.default_joint_vel.clone() * 0.0
        self._warned = set()
        self._bodies = XFormPrim(
            [f"/World/Robot/{name}" for name in robot.body_names],
            reset_xform_properties=False,
        )

        # The workpiece is scenery with no physics: the ROS side owns where it
        # is (resting, or riding the gripper) and this just moves the prop.
        self._workpiece_op = None
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath("/World/workpiece")
        if prim.IsValid():
            for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    self._workpiece_op = op
                    break

    def write_back_to_stage(self):
        self._bodies.set_world_poses(
            self._robot.data.body_pos_w[0], self._robot.data.body_quat_w[0]
        )

    def _get(self, attribute):
        return self._og.Controller.get(self._og.Controller.attribute(attribute))

    def apply(self):
        import torch

        names = self._get("/ROS2/mirror/Joints.outputs:jointNames")
        positions = self._get("/ROS2/mirror/Joints.outputs:positionCommand")
        if names is not None and positions is not None and len(names):
            for name, position in zip(names, positions):
                index = self._index_of.get(name)
                if index is None:
                    if name not in self._warned:
                        self._warned.add(name)
                        print(f"mirror: no joint '{name}' in the articulation", flush=True)
                    continue
                self._positions[0, index] = float(position)
            self._robot.write_joint_state_to_sim(self._positions, self._velocities)

        try:
            px = self._get("/ROS2/mirror/BasePose.outputs:pose:position:x")
            py = self._get("/ROS2/mirror/BasePose.outputs:pose:position:y")
            pz = self._get("/ROS2/mirror/BasePose.outputs:pose:position:z")
            qx = self._get("/ROS2/mirror/BasePose.outputs:pose:orientation:x")
            qy = self._get("/ROS2/mirror/BasePose.outputs:pose:orientation:y")
            qz = self._get("/ROS2/mirror/BasePose.outputs:pose:orientation:z")
            qw = self._get("/ROS2/mirror/BasePose.outputs:pose:orientation:w")
        except Exception as error:  # noqa: BLE001 - attribute layout is version-y
            if "base_pose" not in self._warned:
                self._warned.add("base_pose")
                print(f"mirror: cannot read base pose attributes: {error}", flush=True)
            return
        if qw == 0.0 and qx == 0.0 and qy == 0.0 and qz == 0.0:
            return  # nothing received yet
        root = self._robot.data.default_root_state.clone()
        root[0, 0:3] = torch.tensor([px, py, pz])
        root[0, 3:7] = torch.tensor([qw, qx, qy, qz])
        self._robot.write_root_pose_to_sim(root[:, :7])
        self._robot.write_root_velocity_to_sim(root[:, 7:] * 0.0)

    def _apply_workpiece(self):
        """Place the workpiece prop.

        The message's frame_id says how: "odom" is a world pose, applied as
        is. A link name means the part is rigidly held - its pose is composed
        here from that link's *physics* pose, the same source the hand
        renders from, so the prop rides the gripper without the lag and
        flicker a TF-recomputed world pose had on camera.
        """
        if self._workpiece_op is None:
            return
        from pxr import Gf

        try:
            frame = self._get("/ROS2/mirror/Workpiece.outputs:header:frame_id")
            wx = self._get("/ROS2/mirror/Workpiece.outputs:pose:position:x")
            wy = self._get("/ROS2/mirror/Workpiece.outputs:pose:position:y")
            wz = self._get("/ROS2/mirror/Workpiece.outputs:pose:position:z")
            qx = self._get("/ROS2/mirror/Workpiece.outputs:pose:orientation:x")
            qy = self._get("/ROS2/mirror/Workpiece.outputs:pose:orientation:y")
            qz = self._get("/ROS2/mirror/Workpiece.outputs:pose:orientation:z")
            qw = self._get("/ROS2/mirror/Workpiece.outputs:pose:orientation:w")
        except Exception:  # noqa: BLE001 - absent until first message
            return
        if qw == 0.0 and qx == 0.0 and qy == 0.0 and qz == 0.0:
            return  # nothing received yet

        if frame and frame != "odom":
            index = self._index_of_body(frame)
            if index is None:
                return
            from render_cell import quaternion_matrix
            import numpy as np

            link_pos = self._robot.data.body_pos_w[0, index].cpu().numpy()
            link_rot = quaternion_matrix(
                self._robot.data.body_quat_w[0, index].cpu().numpy()
            )
            rel = np.array([float(wx), float(wy), float(wz)])
            world = link_pos + link_rot @ rel
            self._workpiece_op.Set(Gf.Vec3d(*[float(v) for v in world]))
        else:
            self._workpiece_op.Set(Gf.Vec3d(float(wx), float(wy), float(wz)))

    def _index_of_body(self, name):
        if not hasattr(self, "_body_index_cache"):
            self._body_index_cache = {}
        if name not in self._body_index_cache:
            try:
                self._body_index_cache[name] = self._robot.body_names.index(name)
            except ValueError:
                print(f"mirror: no body '{name}' for the held workpiece", flush=True)
                self._body_index_cache[name] = None
        return self._body_index_cache[name]


def optical_transform(stage, link_path, camera_path):
    """The URDF link -> optical frame transform, for the ROS side to publish.

    The camera prim carries whatever rotation IsaacLab's ROS convention put on
    it; composing that with the USD-camera-to-optical flip (x, -y, -z) gives
    the optical frame in the link's own axes.
    """
    import numpy as np
    from pxr import UsdGeom

    cache = UsdGeom.XformCache()
    link_to_camera = np.array(
        cache.ComputeRelativeTransform(
            stage.GetPrimAtPath(camera_path), stage.GetPrimAtPath(link_path)
        )[0]
    ).T  # row-vector convention -> column-vector matrix

    flip = np.diag([1.0, -1.0, -1.0, 1.0])  # USD camera frame -> optical frame
    matrix = link_to_camera @ flip

    translation = matrix[:3, 3]
    rotation = matrix[:3, :3]
    trace = np.trace(rotation)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        qw, qx, qy, qz = (
            0.25 * s,
            (rotation[2, 1] - rotation[1, 2]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
            (rotation[1, 0] - rotation[0, 1]) / s,
        )
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + rotation[i, i] - rotation[j, j] - rotation[k, k]) * 2
        q = [0.0, 0.0, 0.0, 0.0]
        q[i] = 0.25 * s
        q[j] = (rotation[j, i] + rotation[i, j]) / s
        q[k] = (rotation[k, i] + rotation[i, k]) / s
        qw = (rotation[k, j] - rotation[j, k]) / s
        qx, qy, qz = q
    return translation, (qx, qy, qz, qw)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true", help="Open the Isaac window.")
    parser.add_argument(
        "--lidars",
        nargs="*",
        default=["scan_2d"],
        choices=list(LIDARS),
        help="Which RTX lidars to simulate. Each is render load and, under "
        "many subscribers, crash surface; scan_2d is what Nav2 needs.",
    )
    parser.add_argument("--dock", default="home", choices=list(STATIONS) + ["home"])
    parser.add_argument(
        "--standalone",
        action="store_true",
        help="Publish clock, joint states and the chassis transform from Isaac "
        "instead of mirroring the ROS stack.",
    )
    parser.add_argument(
        "--cameras",
        nargs="*",
        default=["head_d435"],
        choices=list(CAMERAS),
        help="Which cameras to render and publish. Each one costs frame time.",
    )
    parser.add_argument(
        "--no-warehouse", action="store_true", help="Bare ground plane instead."
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Bring the stage up, print diagnostics for a few seconds, exit.",
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

    import paint
    import scenery

    cell = Cell.load()
    # use_fabric=False makes physics write poses back to the USD stage. It is
    # slower, and it is not optional: cameras parented under articulation
    # links render from the *stage* pose, and with Fabric-only physics that is
    # the import posture forever - measured as a camera 30 degrees off where
    # TF put it, which no amount of calibration downstream can explain.
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(
            dt=1.0 / 60.0, device="cuda:0", gravity=(0.0, 0.0, 0.0), use_fabric=False
        )
    )

    # The floor lands on the wheel contact plane: the world frame here is the
    # ROS odom frame, whose origin is base_link. Everything static comes from
    # scenery.build so all three entry points show the same cell.
    stage = omni.usd.get_context().get_stage()
    warehouse_path = None
    if not args.no_warehouse:
        from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

        warehouse_path = f"{ISAAC_NUCLEUS_DIR}/{WAREHOUSE_USD.split('Isaac/', 1)[1]}"
    scenery.build(stage, sim_utils, cell, TEXTURES, warehouse_path)

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
    paint.paint_robot(stage)
    paint.spawn_logo(stage)

    # Plain USD cameras parented under the URDF links. IsaacLab's Camera
    # sensor is deliberately not used here: parented under an articulation
    # link it renders from the link's *authored* pose, not the physics pose
    # (render_cell.py works around the same defect), which put every detection
    # 20 degrees and a quarter metre off. A plain prim rides Fabric and
    # renders from where the link actually is.
    #
    # The URDF camera links point +z out of the lens, but their x axis is not
    # the image's row direction - rendered straight off the link, the world
    # comes out rolled a quarter turn. A real RealSense mounts its imager
    # level, so the camera prim carries the roll that makes the picture
    # upright: view axis along link +z (the x half-turn, since a USD camera
    # looks along its own -z), then -90 deg about the view axis. The optical
    # frame is therefore link * Rz(-90 deg), which is exactly the static
    # transform the ROS side publishes; the start-up print below is the
    # cross-check.
    from pxr import UsdGeom

    cameras = {}
    for name in args.cameras:
        link, intrinsics, (width, height) = CAMERAS[name]
        prim_path = f"/World/Robot/{link}/{name}"
        camera = UsdGeom.Camera.Define(stage, prim_path)
        camera.CreateFocalLengthAttr(intrinsics["focal_length"])
        camera.CreateHorizontalApertureAttr(intrinsics["horizontal_aperture"])
        camera.CreateVerticalApertureAttr(
            intrinsics["horizontal_aperture"] * height / width
        )
        near, far = intrinsics["clipping_range"]
        camera.CreateClippingRangeAttr((near, far))
        xform = UsdGeom.Xformable(camera)
        xform.AddRotateZOp().Set(-90.0)
        xform.AddRotateXOp().Set(180.0)
        cameras[name] = prim_path

    # A fixed observer camera looking across all three stations, published so
    # the ROS side can record the run (there is no GUI in headless mode).
    observer = UsdGeom.Camera.Define(stage, "/World/observer")
    observer.CreateFocalLengthAttr(1.6)
    observer.CreateHorizontalApertureAttr(2.652)
    observer.CreateVerticalApertureAttr(2.652 * 720 / 1280)
    observer.CreateClippingRangeAttr((0.05, 50.0))
    # High diagonal vantage from the feeder side: the machine grew into a
    # full enclosure, and the old ground-level spot behind it filmed nothing
    # but its back panel for the whole cycle.
    eye = (-6.0, -9.8, 6.2)
    target = (1.2, 0.4, 0.4)
    import numpy as np

    view = np.array(eye) - np.array(target)  # USD camera looks along -z
    z_axis = view / np.linalg.norm(view)
    x_axis = np.cross((0.0, 0.0, 1.0), z_axis)
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    matrix = np.eye(4)
    matrix[:3, 0], matrix[:3, 1], matrix[:3, 2], matrix[:3, 3] = x_axis, y_axis, z_axis, eye
    from pxr import Gf

    UsdGeom.Xformable(observer).AddTransformOp().Set(Gf.Matrix4d(matrix.T.tolist()))

    # RTX lidars are created through the sensor command, not a spawner config.
    import omni.kit.commands

    lidar_prims = {}
    for name, (link, config, _, _, _) in LIDARS.items():
        if name not in args.lidars:
            lidar_prims[name] = None
            continue
        ok, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=name,
            parent=f"/World/Robot/{link}",
            config=config,
            translation=(0.0, 0.0, 0.0),
        )
        lidar_prims[name] = str(sensor.GetPath()) if ok else None
        print(f"lidar {name}: {'created ' + lidar_prims[name] if ok else 'FAILED'}", flush=True)

    sim.reset()
    root = robot.data.default_root_state.clone()
    robot.write_root_pose_to_sim(root[:, :7])
    robot.write_root_velocity_to_sim(root[:, 7:])
    robot.write_joint_state_to_sim(
        robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
    )
    robot.reset()

    # The ROS graphs only exist after reset, when the stage is final.
    build_camera_graph("observer", "/World/observer", 1280, 720, "observer")
    for name, prim_path in cameras.items():
        link, _, (width, height) = CAMERAS[name]
        build_camera_graph(name, prim_path, width, height, f"{name}_optical")
        translation, quaternion = optical_transform(
            stage, f"/World/Robot/{link}", prim_path
        )
        print(
            f"static tf {link} -> {name}_optical: "
            f"{translation[0]:.4f} {translation[1]:.4f} {translation[2]:.4f} "
            f"{quaternion[0]:.6f} {quaternion[1]:.6f} {quaternion[2]:.6f} "
            f"{quaternion[3]:.6f}",
            flush=True,
        )

    for name, (_, _, publish_type, topic, frame) in LIDARS.items():
        if lidar_prims[name]:
            build_lidar_graph(name, lidar_prims[name], publish_type, topic, frame)

    mirror = None
    if args.standalone:
        build_robot_graph("/World/Robot")
    else:
        build_mirror_graph()
        mirror = Mirror(robot)

    print("publishing:", flush=True)
    for name in cameras:
        print(
            f"  /{name}/color/image_raw  /{name}/color/camera_info  "
            f"/{name}/depth/image_rect_raw   frame {name}_optical",
            flush=True,
        )
    for name, (_, config, publish_type, topic, _) in LIDARS.items():
        print(f"  /{topic}  ({publish_type}, {config})", flush=True)
    if args.standalone:
        print(f"  /clock  /joint_states  tf {ODOM_FRAME} -> {ROBOT_ROOT_FRAME}", flush=True)
    else:
        print("mirroring /joint_states and /base/pose", flush=True)

    if args.check:
        for _ in range(180):
            sim.step()
            if mirror is not None:
                mirror.apply()
        import omni.graph.core as og

        if mirror is not None:
            node = og.Controller.node("/ROS2/mirror/BasePose")
            print("BasePose outputs:", flush=True)
            for attribute in node.get_attributes():
                if attribute.get_name().startswith("outputs:"):
                    print(f"  {attribute.get_name()}", flush=True)
        print("check done", flush=True)
        os._exit(0)

    print("\nrunning - Ctrl-C to stop", flush=True)
    from render_cell import link_world_pose

    probe_joints = [
        "waist_pitch_joint",
        "thigh_pitch_joint",
        "calf_pitch_joint",
        "head_pitch_joint",
    ]
    probe_ids = [robot.joint_names.index(name) for name in probe_joints]

    step_count = 0
    try:
        while simulation_app.is_running():
            sim.step()
            if mirror is not None:
                mirror.apply()
                # Every third frame, not every frame: bulk USD writes race the
                # render thread ("unlock() called by non-owning thread") and
                # the crash rate tracks the write rate. 20 Hz still looks
                # continuous on camera.
                # The write-back is load-bearing: even with use_fabric
                # False the stage keeps the import pose (probe: physics root
                # moved, stage camera stayed at the origin), and cameras
                # render from the stage. Every third frame is enough for
                # 15 fps video and keeps the USD write rate - the suspected
                # crash surface - at a third.
                if step_count % 3 == 0:
                    mirror.write_back_to_stage()
                    # Same beat as the arm's stage write-back: the held part
                    # must advance exactly when the hand's rendered pose does,
                    # or it leads the hand by up to two frames while the base
                    # drives - which reads as flicker on camera.
                    mirror._apply_workpiece()
                step_count += 1
                if step_count % 600 == 0:
                    # Ground truth for cross-checking against the ROS TF tree:
                    # compare with `ros2 run tf2_ros tf2_echo odom head_d435_Link`.
                    position, rotation = link_world_pose(robot, "head_d435_Link")
                    joints = robot.data.joint_pos[0, probe_ids].cpu().numpy()
                    root = robot.data.root_pos_w[0].cpu().numpy()
                    print(
                        f"probe: head_d435_Link at ({position[0]:.3f}, {position[1]:.3f}, "
                        f"{position[2]:.3f}) z-axis ({rotation[0, 2]:.3f}, {rotation[1, 2]:.3f}, "
                        f"{rotation[2, 2]:.3f}) x-axis ({rotation[0, 0]:.3f}, {rotation[1, 0]:.3f}, "
                        f"{rotation[2, 0]:.3f}) root ({root[0]:.3f}, {root[1]:.3f}) "
                        + " ".join(
                            f"{name.split('_joint')[0]}={value:.3f}"
                            for name, value in zip(probe_joints, joints)
                        ),
                        flush=True,
                    )
                    # What the *renderer* believes: the composed stage
                    # transform of the same prims. If these differ from the
                    # physics pose above, the camera renders from somewhere
                    # else than the TF tree claims.
                    import numpy as np
                    from pxr import Usd, UsdGeom

                    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
                    for label, path in [
                        ("stage link ", "/World/Robot/head_d435_Link"),
                        ("stage cam  ", "/World/Robot/head_d435_Link/head_d435"),
                    ]:
                        prim = stage.GetPrimAtPath(path)
                        if not prim.IsValid():
                            print(f"probe: {label} INVALID PRIM {path}", flush=True)
                            continue
                        matrix = np.array(cache.GetLocalToWorldTransform(prim)).T
                        print(
                            f"probe: {label}({matrix[0, 3]:.3f}, {matrix[1, 3]:.3f}, "
                            f"{matrix[2, 3]:.3f}) z-axis ({matrix[0, 2]:.3f}, "
                            f"{matrix[1, 2]:.3f}, {matrix[2, 2]:.3f}) x-axis "
                            f"({matrix[0, 0]:.3f}, {matrix[1, 0]:.3f}, {matrix[2, 0]:.3f})",
                            flush=True,
                        )
    except KeyboardInterrupt:
        pass
    os._exit(0)


if __name__ == "__main__":
    main()
