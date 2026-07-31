"""The cell's parameter table, shared by every process that works in it.

The demo task, the skill servers and the Isaac stage all describe the same
cell, so they read the same parameters (`config/task.yaml`) through the same
code. A process that declared its own copy of even three of these would drift
from the others the first time someone retuned a bench.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from .scene import FEEDER, MACHINE, OUTFEED, CellLayout

DEFAULTS = {
    "frame_id": "odom",
    "arm_group": "left_arm",
    "body_group": "body",
    "head_group": "head",
    "park_group": "both_arms",
    "body_posture": "work",
    "head_posture": "home",
    "park_posture": "off_arms",
    "tip_link": "left_wrist_roll_Link",
    "attach_link": "left_wrist_roll_Link",
    "touch_links": [
        "leftfinger1_Link",
        "leftfinger2_Link",
        "left_wrist_roll_Link",
        "left_wrist_d405_Link",
    ],
    "gripper_controller": "left_gripper_controller",
    "gripper_joints": ["leftfinger1_joint", "leftfinger2_joint"],
    "gripper_gap_at_zero": 0.095,
    "gripper_max_stroke": 0.0475,
    "gripper_open_gap": 0.085,
    "grasp_squeeze": 0.002,
    "tcp_offset": [-0.0305, -0.0018, -0.2414],
    "grasp_height_offset": 0.01,
    "grasp_yaw": 0.0,
    "carry_tcp": [0.36, 0.24, 0.985],
    "approach_height": 0.12,
    "approach_velocity_scaling": 0.15,
    "ik_attempts": 25,
    "ik_timeout": 0.05,
    "repeat": 1,
    "ground_z": -0.0873,
    "station.feeder_xy": [2.20, -4.00],
    "station.machine_xy": [2.20, 0.00],
    "station.outfeed_xy": [2.20, 4.00],
    "tag.feeder_id": 0,
    "tag.machine_id": 1,
    "tag.outfeed_id": 2,
    "tag.size": 0.08,
    "tag.thickness": 0.002,
    "tag.to_part_xy": [-0.16, 0.0],
    "tag.machine_to_part_xy": [-0.30, -0.10],
    "tag_timeout": 5.0,
    "camera_frame": "head_d435_Link",
    "look_posture": "look_down",
    "handeye_correction.xyz": [0.0, 0.0, 0.0],
    "handeye_correction.rpy": [0.0, 0.0, 0.0],
    "tag_detection.max_range": 3.0,
    "tag_detection.min_range": 0.25,
    "tag_detection.fov": 1.047,
    "tag_detection.position_noise": 0.0,
    "tag_detection.rotation_noise": 0.0,
    "bench.size_xy": [0.70, 1.00],
    "bench.top_z": 0.80,
    "bench.thickness": 0.03,
    "bench.part_inset": 0.12,
    "bench.leg_size": 0.05,
    "bench.leg_inset": 0.03,
    "workpiece.size": [0.07, 0.07, 0.04],
    # Scenery truth only - how the blank lies on the feeder. Nothing in the
    # control stack may read it; the vision detector has to measure the yaw.
    "workpiece.spawn_yaw_deg": 18.0,
    "pocket.clearance": 0.012,
    "pocket.wall_thickness": 0.015,
    "pocket.wall_height": 0.015,
    "dock.offset": [0.44, 0.16],
    "dock.yaw": 0.0,
    "dock.standby_retreat": 0.9,
    "base.linear_speed": 0.5,
    "base.angular_speed": 0.8,
    "base.wheel_radius": 0.0827,
    # One sigma of the navigation stack's docking error. Zero makes the base
    # arrive perfectly, which flatters the tag pipeline into looking correct.
    "base.arrival_position_error": 0.03,
    "base.arrival_yaw_error": 0.03,
    "machining.duration": 8.0,
    "machining.wait_for_trigger": False,
    "dock_poses_file": "",
    "handeye_mount_frame": "head_pitch_Link",
    "handeye_truth.xyz": [0.0, 0.0, 0.0],
    "handeye_truth.rpy": [0.0, 0.0, 0.0],
    "handeye_base_dx": [0.0, -0.15, 0.10, -0.10, 0.05, -0.05],
    "handeye_base_dy": [0.0, 0.12, -0.12, -0.08, 0.15, -0.15],
    "handeye_base_dyaw": [0.0, 0.15, -0.15, 0.25, -0.25, 0.10],
    "handeye_head_yaw": [0.0, 0.25, -0.25, 0.15, -0.15, 0.0],
    "handeye_head_pitch": [0.52, 0.30, 0.45, 0.15, 0.52, 0.35],
    "handeye_waist_pitch": [-0.45, -0.10, -0.50, 0.20, 0.10, -0.30],
    "handeye_settle": 0.4,
    "dock_sweep.span": 0.20,
    "dock_sweep.step": 0.05,
    "dock_sweep.yaw_errors": [0.0, 10.0],
    "reach_map.x_range": [0.28, 0.60],
    "reach_map.y_range": [-0.24, 0.42],
    "reach_map.step": 0.04,
}


def declare_cell_parameters(node):
    for name, value in DEFAULTS.items():
        node.declare_parameter(name, value)


def layout_from(node):
    """The cell layout as the node's parameters currently describe it."""
    get = lambda name: node.get_parameter(name).value  # noqa: E731
    return CellLayout(
        frame_id=get("frame_id"),
        ground_z=get("ground_z"),
        station_xy={
            FEEDER: tuple(get("station.feeder_xy")),
            MACHINE: tuple(get("station.machine_xy")),
            OUTFEED: tuple(get("station.outfeed_xy")),
        },
        tag_ids={
            FEEDER: get("tag.feeder_id"),
            MACHINE: get("tag.machine_id"),
            OUTFEED: get("tag.outfeed_id"),
        },
        tag_size=get("tag.size"),
        tag_thickness=get("tag.thickness"),
        tag_to_part_xy=tuple(get("tag.to_part_xy")),
        tag_to_part_overrides={MACHINE: tuple(get("tag.machine_to_part_xy"))},
        bench_size_xy=tuple(get("bench.size_xy")),
        bench_top_z=get("bench.top_z"),
        bench_thickness=get("bench.thickness"),
        bench_part_inset=get("bench.part_inset"),
        leg_size=get("bench.leg_size"),
        leg_inset=get("bench.leg_inset"),
        workpiece_size=tuple(get("workpiece.size")),
        pocket_clearance=get("pocket.clearance"),
        pocket_wall_thickness=get("pocket.wall_thickness"),
        pocket_wall_height=get("pocket.wall_height"),
        dock_offset=tuple(get("dock.offset")),
        dock_yaw=get("dock.yaw"),
        standby_retreat=get("dock.standby_retreat"),
    )


def default_dock_poses_file(node):
    """The taught docking poses, honouring the `dock_poses_file` override."""
    configured = node.get_parameter("dock_poses_file").value
    if configured:
        return Path(configured)
    return (
        Path(get_package_share_directory("franzi_pick_place"))
        / "config"
        / "dock_poses.yaml"
    )


def build_moveit():
    """Start an embedded MoveIt instance that shares move_group's scene."""
    from moveit.planning import MoveItPy
    from moveit_configs_utils import MoveItConfigsBuilder

    moveit_config = (
        MoveItConfigsBuilder("wheel_robot_4.0", package_name="franzi_moveit_config")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
        )
        .pilz_cartesian_limits()
        .to_moveit_configs()
    )

    config = moveit_config.to_dict()
    planning_params = (
        Path(get_package_share_directory("franzi_pick_place")) / "config" / "moveit_py.yaml"
    )
    config.update(yaml.safe_load(planning_params.read_text()))

    return MoveItPy(node_name="pick_place_moveit", config_dict=config)
