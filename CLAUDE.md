# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Most documentation in this repo (and communication with the user) is in Chinese.

## What this repo is

Simulation of the Franzi mobile manipulator for an engraving-machine tending cell
(移动机械臂雕刻机自动上下料), organized by simulation runtime:

- `Rviz/` — the ROS 2 Jazzy colcon workspace. **All robot logic lives here**; the
  control stack is identical across backends.
- `IssacSim/` — NVIDIA Isaac Sim scripts (keep the existing spelling). Isaac's role is
  *sensors and world only*: it mirrors `/joint_states` + `/base/pose` from the ROS stack
  and publishes rendered cameras (`/head_d435/color/…`) and RTX lidar (`/scan`,
  `/mid360/points`). It runs no control.
- `Mujoco/` — placeholder, empty.

## Architecture (Rviz/src)

Layered so each layer can be swapped for real hardware without touching the others
(the migration table is in `Rviz/src/franzi_skills/README.md`):

```
franzi_task_manager       task layer: data-driven state machine (states.yaml)
        │  8 ROS 2 actions
franzi_skills             skill layer: navigate / precise-dock / detect / pick / load / unload / place / safe-pose
        │  MoveItPy · TF2 · AprilTag · base
franzi_pick_place         component library: MobileBase, ArmMotion, Gripper, planning scene, tag pipeline
        │
franzi_machine_bridge     engraving-machine bridge: interlocks, timeouts, simulated PLC
franzi_engraving_interfaces   msg / srv / action definitions
franzi_description        URDF + meshes;  franzi_moveit_config  MoveIt 2 config
franzi_sdk                vendor SDK for the real robot (not used in simulation)
```

Key invariants:

- **`Rviz/src/franzi_pick_place/config/task.yaml` is the single source of truth for the
  cell layout.** Isaac's `cell.py` reads it directly; nothing in code hardcodes a pose.
- The chassis is holonomic; the SRDF `world_joint` is `planar` and the base pose arrives
  as a `world -> moveit_root` TF published by `base.py`. Without that transform MoveIt's
  current state never completes.
- `robot/signals` has exactly one writer (the task manager); interlock semantics are
  declared in the state table (`signals_on_entry` / `signals_on_success`).
- Precise docking decisions are pure functions in `dock_controller.py` with the executor
  injected — simulation uses kinematic `drive_to(exact=True)`, hardware will use a
  cmd_vel loop.
- Isaac backend swaps only perception: mock detector off, apriltag_ros decodes the real
  rendered image and publishes the same `tag_N` TF frames. Control is untouched.

## Build

From `Rviz/`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  franzi_engraving_interfaces franzi_machine_bridge \
  franzi_task_manager franzi_skills franzi_pick_place \
  franzi_description franzi_moveit_config
```

Then `source install/setup.bash` in each new terminal.

## Run

Full tending cycle, RViz backend (geometric sim, mock detection):

```bash
ros2 launch franzi_skills tending_cell.launch.py
```

Useful args: `rviz:=false`, `cycles:=2`, `autostart:=false`, `task_delay:=60.0`
(the task manager waits ~40 s by default for MoveItPy to come up).

Isaac backend (real rendering + real AprilTag detection) — two processes:

```bash
IssacSim/run_ros2_cell.sh                                  # Isaac mirror, terminal 1
ros2 launch franzi_skills tending_cell.launch.py backend:=isaac   # ROS stack, terminal 2
```

`run_ros2_cell.sh` also takes `--standalone --gui` (scene only) and `--check`
(self-test then exit). Wrist camera streams need `--cameras head_d435 left_wrist_d405`.

MoveIt-only visualization: `ros2 launch franzi_moveit_config demo.launch.py`
(`publish_demo_joint_states:=false` when something else owns `/joint_states`).

## Test

From `Rviz/`:

```bash
python3 -m pytest src/franzi_machine_bridge/test src/franzi_task_manager/test src/franzi_skills/test
```

Single test: `python3 -m pytest src/franzi_skills/test/test_dock_controller.py -k <name>`.
The pure-logic modules (`interlocks.py`, `state_table.py`, `dock_controller.py`,
`slots.py`) test without ROS running.

## Environment pitfalls (all previously hit on this machine)

- **Python shadowing breaks colcon/rosidl.** Deactivate conda before building
  (`ModuleNotFoundError: catkin_pkg`), and remove uv-installed Python from `PATH`
  (`~/.local/bin`) before building interface packages (`ModuleNotFoundError: em`).
- GUI/ROS launches may need a clean shell:
  `env -u LD_LIBRARY_PATH -u LD_PRELOAD -u LD_AUDIT bash -lc '…'`.
- The Isaac process keeps its own Python (`env_isaaclab` conda env) and only borrows
  ROS *C* libraries — it cannot import system rclpy; all its subscriptions go through
  OmniGraph nodes. Never `source setup.bash` wholesale for Isaac; use
  `run_ros2_cell.sh`.
- Isaac-specific traps (cameras under articulation links don't follow physics, AprilTag
  vs ArUco 180° frame convention, RTX lidar nested prim, `useSystemTime` stamps) are
  documented in `IssacSim/README.md` — read it before touching Isaac code.

## Conventions

- Commit subjects: short, lowercase, imperative (`add the warehouse and wire up the lidars`).
- 不要生成播放器打不开的视频文件:OpenCV 写出的 mp4v 编码很多播放器不认,交付前
  必须转成 H.264(`ffmpeg -i in.mp4 -c:v libx264 -pix_fmt yuv420p -movflags +faststart out.mp4`)。
- Keep engine-specific files in their runtime directory; no machine-specific absolute
  paths in code — keep simulator paths and launch parameters configurable.
- New executable code must come with a reproducible validation command in the owning
  README. There is no CI; the READMEs' run commands are the validation record.
- `Rviz/NextStep.md` documents the plan for driving the real robot
  (`franzi_hardware_bridge` over `franzi_sdk`); the physics stage plan is at the end of
  `IssacSim/README.md`.
