"""Tests for the runtime-neutral box-transfer state contract."""

from pathlib import Path
from xml.etree import ElementTree

from franzi_sim.scenarios.box_transfer import (
    BOX_TRANSFER_CAMERA_REVIEW_POSE_RAD,
    BoxTransferConfig,
    BoxTransferStateMachine,
    TransferStage,
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
