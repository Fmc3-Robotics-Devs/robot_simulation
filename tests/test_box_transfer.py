"""Tests for the runtime-neutral box-transfer state contract."""

from pathlib import Path
from xml.etree import ElementTree

from franzi_sim.scenarios.box_transfer import (
    BOX_TRANSFER_CAMERA_REVIEW_POSE_RAD,
    BoxTransferConfig,
    BoxTransferStateMachine,
    TransferStage,
)
from franzi_sim.scenarios.box_transfer_demo import (
    BASE_Z_M,
    BOX_INITIAL_POSITION_M,
    BOX_TARGET_POSITION_M,
    DEMO_SEGMENTS,
    demo_duration_s,
    sample_demo,
)


def test_state_machine_completes_in_order() -> None:
    """Every successful executor acknowledgement advances exactly one stage."""

    machine = BoxTransferStateMachine(BoxTransferConfig())
    while not machine.is_terminal:
        machine.complete_stage()
    assert machine.stage is TransferStage.COMPLETE


def test_state_machine_retries_then_fails() -> None:
    """Failure never advances the task past the unverified stage."""

    machine = BoxTransferStateMachine(BoxTransferConfig(max_retries=1))
    assert machine.fail_stage("tag missing") is TransferStage.SEARCH_TAG
    assert machine.retries == 1
    assert machine.fail_stage("tag missing") is TransferStage.FAILED
    assert machine.failure_reason == "tag missing"


def test_reset_clears_runtime_state() -> None:
    """Reset supports deterministic batch runs using one configuration."""

    machine = BoxTransferStateMachine(BoxTransferConfig(max_retries=0))
    machine.fail_stage("planner timeout")
    machine.reset()
    assert machine.stage is TransferStage.SEARCH_TAG
    assert machine.retries == 0
    assert machine.failure_reason is None


def test_camera_review_pose_is_deterministic_and_dual_arm() -> None:
    """The evidence pose keeps both arms in the mirrored neutral configuration."""

    positions = dict(BOX_TRANSFER_CAMERA_REVIEW_POSE_RAD)
    assert len(positions) == len(BOX_TRANSFER_CAMERA_REVIEW_POSE_RAD) == 14
    assert set(positions.values()) == {0.0}
    for left_name, left_position in positions.items():
        if not left_name.startswith("left_"):
            continue
        right_name = left_name.replace("left_", "right_", 1)
        assert positions[right_name] == left_position


def test_camera_review_pose_respects_urdf_joint_limits() -> None:
    """A visually useful evidence pose must also be executable by the robot."""

    urdf = ElementTree.parse(
        Path("Rviz/src/franzi_description/urdf/wheel_robot_4.0.urdf")
    )
    limits = {
        joint.attrib["name"]: (
            float(limit.attrib["lower"]),
            float(limit.attrib["upper"]),
        )
        for joint in urdf.getroot().findall("joint")
        if (limit := joint.find("limit")) is not None
    }
    for joint_name, position_rad in BOX_TRANSFER_CAMERA_REVIEW_POSE_RAD:
        lower_rad, upper_rad = limits[joint_name]
        assert lower_rad <= position_rad <= upper_rad


def test_demo_keyframes_are_continuous_and_limit_compliant() -> None:
    """Every transition joins exactly and all authored joints respect the URDF."""

    urdf = ElementTree.parse(
        Path("Rviz/src/franzi_description/urdf/wheel_robot_4.0.urdf")
    )
    limits = {
        joint.attrib["name"]: (
            float(limit.attrib["lower"]),
            float(limit.attrib["upper"]),
        )
        for joint in urdf.getroot().findall("joint")
        if (limit := joint.find("limit")) is not None
        and "lower" in limit.attrib
        and "upper" in limit.attrib
    }
    for previous, following in zip(DEMO_SEGMENTS, DEMO_SEGMENTS[1:], strict=False):
        assert previous.end == following.start
    for segment in DEMO_SEGMENTS:
        assert segment.duration_s > 0.0
        for pose in (segment.start, segment.end):
            assert pose.base_position_m[2] == BASE_Z_M
            for joint_name, position in pose.joint_positions:
                lower, upper = limits[joint_name]
                assert lower <= position <= upper


def test_demo_moves_box_between_pick_table_corners() -> None:
    """The recorded route starts and ends at distinct supported table corners."""

    assert BOX_INITIAL_POSITION_M == (1.20, 0.38, 0.995)
    assert BOX_TARGET_POSITION_M == (1.20, -0.72, 0.995)
    assert BOX_TARGET_POSITION_M[1] < -0.5
    assert demo_duration_s() == 21.0


def test_demo_sampler_exposes_attach_lift_transport_and_release() -> None:
    """The sampled timeline makes controlled attachment boundaries explicit."""

    samples = [sample_demo(index * 0.05) for index in range(round(demo_duration_s() / 0.05) + 1)]
    carried = [sample for sample in samples if sample.carries_box]
    assert carried[0].segment_name == "lift"
    assert carried[-1].segment_name == "release"
    assert any(sample.segment_name == "transport" for sample in carried)
    assert max(sample.base_position_m[0] for sample in samples) == 0.618
    assert sample_demo(demo_duration_s()).segment_name == "complete"


def test_demo_continuous_wheel_targets_remain_inside_physx_drive_range() -> None:
    """Equivalent wrapped angles avoid PhysX rejecting long-route targets."""

    for index in range(round(demo_duration_s() / 0.01) + 1):
        joints = sample_demo(index * 0.01).joint_map()
        wheel_targets = [
            position
            for name, position in joints.items()
            if name.endswith("_wheel_joint")
        ]
        assert wheel_targets
        assert max(abs(position) for position in wheel_targets) <= 3.141593


def test_demo_opens_then_withdraws_before_folding_arms() -> None:
    """Release must not let the joint-space return arc brush the delivered box."""

    release = next(segment for segment in DEMO_SEGMENTS if segment.name == "release")
    withdraw = next(segment for segment in DEMO_SEGMENTS if segment.name == "withdraw")
    retreat = next(segment for segment in DEMO_SEGMENTS if segment.name == "retreat")

    assert release.carries_box is True
    assert withdraw.carries_box is False
    assert withdraw.start.joint_positions == withdraw.end.joint_positions
    assert withdraw.end.base_position_m[0] < withdraw.start.base_position_m[0]
    assert retreat.start == withdraw.end
