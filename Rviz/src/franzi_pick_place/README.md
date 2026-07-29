# Franzi pick-and-place (machine tending)

Kinematic machine-tending demo for the Franzi robot on ROS 2 Jazzy. Three benches
stand 4 m apart, so the mobile base is part of the job:

1. dock at the feeder bench and pick the workpiece,
2. drive to the machine and insert it into the fixture pocket,
3. back off, wait for the machining cycle (status light in RViz),
4. drive back in, pick the finished part out,
5. drive to the outfeed bench and set it down.

Arm motion is planned by MoveIt against a collision world made of boxes;
grasping is modelled by attaching the workpiece to the wrist in the planning
scene. The chassis pose is integrated, not planned. No physics engine is
involved, so contact forces and real insertion tolerances are explicitly
**not** validated here — see [Limitations](#limitations).

## Build

From the `Rviz/` workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select franzi_description franzi_moveit_config franzi_pick_place
```

Requires `ros-jazzy-moveit-py` in addition to the packages MoveIt pulls in.

## Run

```bash
env -u LD_LIBRARY_PATH -u LD_PRELOAD -u LD_AUDIT bash -lc \
  'source /opt/ros/jazzy/setup.bash && source install/setup.bash \
   && QT_QPA_PLATFORM=xcb ros2 launch franzi_pick_place pick_place_demo.launch.py'
```

This starts move_group, the virtual trajectory controllers, RViz with the cell
and the status light, and then the task node. Useful arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `use_rviz` | `true` | Set `false` for a headless run. |
| `task_config` | `config/task.yaml` | Alternative cell / task parameter file. |
| `start_delay` | `6.0` | Seconds to wait for move_group before starting. |

Run the task on its own against an already-running move_group:

```bash
ros2 run franzi_pick_place pick_place_task --ros-args \
  --params-file <install>/share/franzi_pick_place/config/task.yaml
```

## The machining handshake

The station publishes its state on `/machining_station/markers` (grey = idle,
blinking red = machining, green = done) and the robot only picks the part back
out once the cycle is over. By default the cycle is a fixed
`machining.duration`. To drive it from outside instead, set
`machining.wait_for_trigger: true` and end the cycle with:

```bash
ros2 service call /machining_station/complete std_srvs/srv/Trigger {}
```

That is the hook to replace with a real machine interface later.

## Motion strategy

Transfers between stations are free-space OMPL (`RRTConnect`) plans; the 12 cm
approach, insertion and retreat segments are Pilz `LIN` straight lines. A full
cycle is 8 of each, with an automatic fallback to free space if a straight line
cannot be found — watch for `no straight-line plan` in the log, it means that
segment is no longer a controlled vertical approach.

Every goal is reduced to a joint configuration before planning. The arm is
redundant, so a pose has a continuum of solutions; the contact pose is solved
first and its approach pose is then seeded from it, which keeps both on the same
IK branch. Solving them independently produces postures 12 cm apart that the arm
cannot travel between in a straight line, and Pilz rejects the segment.

## The mobile base

The chassis is a three-wheel platform with all three wheels independently
steered, so it is holonomic — it strafes between benches without turning. The
SRDF's `world_joint` is therefore **`planar`**, not fixed, and the base pose
arrives as a `world -> moveit_root` transform. Nothing else works without it:
with no transform the planar joint has no value and MoveIt's current state
never completes; with a stale one it plans as though the robot were still
parked at the origin.

`base.py` owns both halves of that: the transform, and the six steering/wheel
joint states (it solves the swerve kinematics, so the wheels in RViz point where
the platform is actually going). `franzi_moveit_config`'s demo driver therefore
has to give those up — `demo.launch.py` grew `publish_virtual_joint_tf` and
`publish_chassis_joints` arguments, both defaulting to the old behaviour, and
this package's launch file sets both to `false`.

On hardware the same two arguments are what you turn off to let localisation and
the real chassis driver take over.

## Cell layout

All poses are in the `world` frame. The floor (wheel contact) is at
`z = -0.0873`, so the default bench top at `z = 0.80` is 0.887 m above the
ground. Retune everything in `config/task.yaml`; nothing in the code hardcodes
a pose.

Stations are laid out along `y` so the base strafes between them:

| Station | world x, y | Role |
| --- | --- | --- |
| feeder | 2.20, -4.00 | where the raw workpiece starts |
| machine | 2.20, 0.00 | fixture the part is inserted into |
| outfeed | 2.20, 4.00 | where the finished part is set down |

The odom origin - where the base wakes up - is the robot's home spot,
1.8 m back from the dock line, mid-aisle. The machine's tag has its own
surveyed offset (`tag.machine_to_part_xy`): the fixture sits under the
spindle, so the marker moves deeper onto the pallet plate, and it must stay
within the head camera's ~34 degree half-FOV as seen from the dock.

The station world poses and the arm envelope are tied together by one number,
`dock.offset` (default `[0.44, 0.16]`): where a station has to sit **in the base
frame** once parked. The dock pose is derived from it, so moving a bench never
invalidates the arm plans — only changing `dock.offset` does.

Two constraints pull `dock.offset` in opposite directions: too large and the
station falls outside the arm envelope; too small and the chassis (which reaches
to `x = +0.24`) parks inside the bench. The demo checks the second case for you —
after every dock it verifies the robot is not in collision and fails loudly if
it is.

## Checking reachability before moving a station

The binding constraint is not arm length, it is self-collision: reaching across
the body midline makes the upper arm hit the torso (`torso_Link` vs
`left_shoulder_roll_Link` / `left_shoulder_yaw_Link`, which the SRDF
deliberately does not whitelist). The envelope closes in quickly past `y = 0` —
at `x = 0.40` the last reachable row is around `y = -0.12`, and it retreats
further as `x` grows.

Sample the envelope instead of guessing. With move_group already running:

```bash
ros2 launch franzi_moveit_config demo.launch.py use_rviz:=false   # terminal 1
ros2 run franzi_pick_place reach_map --ros-args \
  --params-file install/franzi_pick_place/share/franzi_pick_place/config/task.yaml
```

It prints a `+` / `.` grid of collision-free top-down grasps **in the base
frame**, at the grasp and approach heights, with a bench placed where
`dock.offset` says a station will be. Keep `dock.offset` away from the boundary
of the `+` region: a pose near the edge is still reachable, but the straight-line
approach to it often is not, and the segment silently degrades to a free-space
detour.

## Gripper geometry

Worth knowing before changing grasp parameters: the two finger joints sit 95 mm
apart in `x` on the wrist and travel *towards* each other, so

```
gap = 0.095 - (finger1 - finger2)      finger1 in [0, 0.0475], finger2 in [-0.0475, 0]
```

Joint zero is fully open (95 mm) and the travel limits are fully closed. The
`open` / `closed` named states in the SRDF were swapped relative to this and
have been corrected.

The grasp centre between the finger pads is at `(-0.0305, -0.0018, -0.2414)` in
`left_wrist_roll_Link`, which is the `tcp_offset` parameter. The approach axis
is the tip link's `-z`, so a zero-rotation goal means a top-down grasp.

## Limitations

- **No physics.** The workpiece follows the wrist exactly; it cannot slip,
  tip over or jam. A grasp that would fail on hardware still succeeds here.
- **Insertion tolerance is not tested.** `pocket.clearance` defaults to 12 mm
  per side. Tightening it makes planning brittle without telling you anything
  about the real assembly — that belongs in the physics stage.
- **Workpiece contacts are whitelisted.** Collisions between the workpiece and
  the benches / fixture are allowed in the ACM, because a part resting on a
  surface is contact by definition. Robot links are still checked against both.
- **The base is driven, not navigated.** Docking is a straight line in the world
  plane with no path planning, no obstacle avoidance and no localisation error;
  the route between benches is never collision-checked, only the parked pose is.
  Anything put in the aisle will be driven straight through. Nav2 is the
  replacement when that matters.
- **The chassis is assumed holonomic.** If the real platform turns out to be
  non-holonomic, `MobileBase.drive_to` is where that changes — the docking poses
  and the task sequence stay as they are.
