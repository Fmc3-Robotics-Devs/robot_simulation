"""Deterministic whole-body keyframes for the recorded box-transfer demo.

The demo is intentionally a software-in-the-loop proof of the task sequence.
The mobile base is moved kinematically, the imported articulation is keyframed,
and the box uses a verified controlled attachment between grasp and release.
This keeps the visual result reproducible while the ROS 2/MoveIt executors are
still scheduled for the later control-integration gate.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


BASE_Z_M = 0.0873
BOX_INITIAL_POSITION_M = (1.20, 0.38, 0.995)
BOX_TARGET_POSITION_M = (1.20, -0.72, 0.995)
BOX_TRANSPORT_CLEARANCE_M = 0.28
WHEEL_RADIUS_M = 0.116

ARM_JOINT_SUFFIXES = (
    "shoulder_pitch_joint",
    "shoulder_roll_joint",
    "shoulder_yaw_joint",
    "elbow_pitch_joint",
    "wrist_yaw_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
)
BODY_JOINTS = (
    "calf_pitch_joint",
    "thigh_pitch_joint",
    "waist_pitch_joint",
    "waist_yaw_joint",
    "head_yaw_joint",
    "head_pitch_joint",
)
FINGER_JOINTS = (
    "leftfinger1_joint",
    "leftfinger2_joint",
    "rightfinger1_joint",
    "rightfinger2_joint",
)
STEERING_JOINTS = (
    "left_front_steering_joint",
    "right_front_steering_joint",
    "rear_steering_joint",
)
WHEEL_JOINTS = (
    "left_front_wheel_joint",
    "right_front_wheel_joint",
    "rear_wheel_joint",
)

HOME_ARM_RAD = (0.0,) * len(ARM_JOINT_SUFFIXES)
# The reduced shoulder-roll magnitude brings both grip points to within 13 mm
# of the blue-box side targets while preserving each wrist/tool -Z direction.
PREGRASP_LEFT_ARM_RAD = (0.590, 0.302, -0.467, -1.538, -1.092, 0.409, 0.826)
PREGRASP_RIGHT_ARM_RAD = (0.590, -0.302, 0.467, -1.538, 1.092, 0.409, -0.826)
LIFT_LEFT_ARM_RAD = (0.534, 0.438, -0.130, -2.515, -1.401, 0.425, -0.345)
LIFT_RIGHT_ARM_RAD = (0.534, -0.438, 0.130, -2.515, 1.401, 0.425, 0.345)


@dataclass(frozen=True)
class DemoPose:
    """One complete articulation and mobile-base pose."""

    name: str
    base_position_m: tuple[float, float, float]
    joint_positions: tuple[tuple[str, float], ...]

    def joint_map(self) -> dict[str, float]:
        """Return the immutable joint tuple as a convenient mapping."""

        return dict(self.joint_positions)


@dataclass(frozen=True)
class DemoSegment:
    """A quintic transition between two poses with one task-stage label."""

    name: str
    duration_s: float
    start: DemoPose
    end: DemoPose
    carries_box: bool


@dataclass(frozen=True)
class DemoSample:
    """Interpolated command and task metadata at one video timestamp."""

    time_s: float
    segment_name: str
    segment_progress: float
    overall_progress: float
    base_position_m: tuple[float, float, float]
    joint_positions: tuple[tuple[str, float], ...]
    carries_box: bool

    def joint_map(self) -> dict[str, float]:
        """Return sampled positions keyed by articulation DOF name."""

        return dict(self.joint_positions)


def _joint_map(
    *,
    left_arm: tuple[float, ...] = HOME_ARM_RAD,
    right_arm: tuple[float, ...] = HOME_ARM_RAD,
    fingers_closed: bool = False,
    head_pitch_rad: float = 0.0,
) -> dict[str, float]:
    """Build a full deterministic upper-body command for one pose."""

    values = {name: 0.0 for name in BODY_JOINTS}
    values["head_pitch_joint"] = head_pitch_rad
    values.update(
        {
            f"left_{suffix}": value
            for suffix, value in zip(ARM_JOINT_SUFFIXES, left_arm, strict=True)
        }
    )
    values.update(
        {
            f"right_{suffix}": value
            for suffix, value in zip(ARM_JOINT_SUFFIXES, right_arm, strict=True)
        }
    )
    finger_offset = 0.032 if fingers_closed else 0.0
    values.update(
        {
            "leftfinger1_joint": finger_offset,
            "leftfinger2_joint": -finger_offset,
            "rightfinger1_joint": finger_offset,
            "rightfinger2_joint": -finger_offset,
        }
    )
    return values


def _pose(
    name: str,
    base_xy_m: tuple[float, float],
    *,
    left_arm: tuple[float, ...] = HOME_ARM_RAD,
    right_arm: tuple[float, ...] = HOME_ARM_RAD,
    fingers_closed: bool = False,
    head_pitch_rad: float = 0.0,
) -> DemoPose:
    """Create a pose while keeping the measured wheel-contact height fixed."""

    joints = _joint_map(
        left_arm=left_arm,
        right_arm=right_arm,
        fingers_closed=fingers_closed,
        head_pitch_rad=head_pitch_rad,
    )
    return DemoPose(
        name=name,
        base_position_m=(base_xy_m[0], base_xy_m[1], BASE_Z_M),
        joint_positions=tuple(joints.items()),
    )


HOME = _pose("home", (0.0, 0.0))
APPROACH = _pose("approach", (0.618, 0.38), head_pitch_rad=0.18)
PREGRASP = _pose(
    "pregrasp",
    (0.618, 0.38),
    left_arm=PREGRASP_LEFT_ARM_RAD,
    right_arm=PREGRASP_RIGHT_ARM_RAD,
    head_pitch_rad=0.18,
)
GRASP = _pose(
    "grasp",
    (0.618, 0.38),
    left_arm=PREGRASP_LEFT_ARM_RAD,
    right_arm=PREGRASP_RIGHT_ARM_RAD,
    fingers_closed=True,
    head_pitch_rad=0.18,
)
LIFT = _pose(
    "lift",
    (0.518, 0.38),
    left_arm=LIFT_LEFT_ARM_RAD,
    right_arm=LIFT_RIGHT_ARM_RAD,
    fingers_closed=True,
    head_pitch_rad=0.12,
)
TRANSPORT = _pose(
    "transport",
    (0.518, -0.72),
    left_arm=LIFT_LEFT_ARM_RAD,
    right_arm=LIFT_RIGHT_ARM_RAD,
    fingers_closed=True,
    head_pitch_rad=0.12,
)
LOWER = _pose(
    "lower",
    (0.618, -0.72),
    left_arm=PREGRASP_LEFT_ARM_RAD,
    right_arm=PREGRASP_RIGHT_ARM_RAD,
    fingers_closed=True,
    head_pitch_rad=0.18,
)
RELEASE = _pose(
    "release",
    (0.618, -0.72),
    left_arm=PREGRASP_LEFT_ARM_RAD,
    right_arm=PREGRASP_RIGHT_ARM_RAD,
    head_pitch_rad=0.18,
)
WITHDRAW = _pose(
    "withdraw",
    (0.350, -0.72),
    left_arm=PREGRASP_LEFT_ARM_RAD,
    right_arm=PREGRASP_RIGHT_ARM_RAD,
    head_pitch_rad=0.18,
)
RETREAT = _pose("retreat", (0.25, -0.72), head_pitch_rad=0.08)

DEMO_SEGMENTS = (
    DemoSegment("home", 0.75, HOME, HOME, False),
    DemoSegment("approach", 2.50, HOME, APPROACH, False),
    DemoSegment("pregrasp", 2.50, APPROACH, PREGRASP, False),
    DemoSegment("grasp", 1.00, PREGRASP, GRASP, False),
    DemoSegment("lift", 2.25, GRASP, LIFT, True),
    DemoSegment("transport", 3.00, LIFT, TRANSPORT, True),
    DemoSegment("lower", 2.25, TRANSPORT, LOWER, True),
    # Keep the controlled hold until the fingers are fully open.  The runtime
    # removes it at the first retreat sample, before the base moves away.
    DemoSegment("release", 1.00, LOWER, RELEASE, True),
    # Pull the open hands straight away with the mobile base before folding
    # the arms.  This prevents the joint-space return arc from brushing the
    # released box and changing its final orientation.
    DemoSegment("withdraw", 1.25, RELEASE, WITHDRAW, False),
    DemoSegment("retreat", 1.25, WITHDRAW, RETREAT, False),
    DemoSegment("return_home", 2.50, RETREAT, HOME, False),
    DemoSegment("complete", 0.75, HOME, HOME, False),
)


def quintic_blend(progress: float) -> float:
    """Return a zero-velocity, zero-acceleration interpolation weight."""

    value = min(1.0, max(0.0, progress))
    return 10.0 * value**3 - 15.0 * value**4 + 6.0 * value**5


def demo_duration_s() -> float:
    """Return the complete recorded trajectory duration."""

    return sum(segment.duration_s for segment in DEMO_SEGMENTS)


def _interpolate_tuple(
    start: tuple[float, ...],
    end: tuple[float, ...],
    weight: float,
) -> tuple[float, ...]:
    return tuple(first + (last - first) * weight for first, last in zip(start, end, strict=True))


def _wheel_state(time_s: float) -> tuple[float, float]:
    """Return visual swerve steering and wheel roll for the sampled time."""

    elapsed = 0.0
    cumulative_distance = 0.0
    steering_rad = 0.0
    for segment in DEMO_SEGMENTS:
        delta_x = segment.end.base_position_m[0] - segment.start.base_position_m[0]
        delta_y = segment.end.base_position_m[1] - segment.start.base_position_m[1]
        distance = math.hypot(delta_x, delta_y)
        if time_s <= elapsed + segment.duration_s:
            local = (time_s - elapsed) / segment.duration_s
            weight = quintic_blend(local)
            cumulative_distance += distance * weight
            if distance > 1e-9:
                # Return steering toward straight-ahead near both endpoints so
                # the swerve pods visibly prepare and settle around each move.
                travel_heading = math.atan2(delta_y, delta_x)
                edge_weight = min(1.0, local / 0.18, (1.0 - local) / 0.18)
                steering_rad = travel_heading * quintic_blend(edge_weight)
            break
        cumulative_distance += distance
        elapsed += segment.duration_s
    # PhysX continuous revolute drives still reject targets outside [-2π, 2π].
    # Wrapping is visually identical and keeps a long mobile-base path valid.
    wheel_rotation_rad = math.remainder(
        -cumulative_distance / WHEEL_RADIUS_M,
        2.0 * math.pi,
    )
    return steering_rad, wheel_rotation_rad


def sample_demo(time_s: float) -> DemoSample:
    """Sample the complete keyframed command at an absolute demo time."""

    duration = demo_duration_s()
    clamped_time = min(duration, max(0.0, time_s))
    elapsed = 0.0
    segment = DEMO_SEGMENTS[-1]
    local_progress = 1.0
    for candidate in DEMO_SEGMENTS:
        end_time = elapsed + candidate.duration_s
        if clamped_time <= end_time:
            segment = candidate
            local_progress = (clamped_time - elapsed) / candidate.duration_s
            break
        elapsed = end_time
    weight = quintic_blend(local_progress)
    base_position = _interpolate_tuple(
        segment.start.base_position_m,
        segment.end.base_position_m,
        weight,
    )
    start_joints = segment.start.joint_map()
    end_joints = segment.end.joint_map()
    joints = {
        name: start_joints[name] + (end_joints[name] - start_joints[name]) * weight
        for name in start_joints
    }
    steering_rad, wheel_rotation_rad = _wheel_state(clamped_time)
    joints.update({name: steering_rad for name in STEERING_JOINTS})
    joints.update({name: wheel_rotation_rad for name in WHEEL_JOINTS})
    return DemoSample(
        time_s=clamped_time,
        segment_name=segment.name,
        segment_progress=local_progress,
        overall_progress=clamped_time / duration,
        base_position_m=base_position,
        joint_positions=tuple(joints.items()),
        carries_box=segment.carries_box,
    )
