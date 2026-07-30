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
    """The evidence pose drives both arms and preserves the reviewed D405 view."""

    positions = dict(BOX_TRANSFER_CAMERA_REVIEW_POSE_RAD)
    assert len(positions) == len(BOX_TRANSFER_CAMERA_REVIEW_POSE_RAD) == 12
    assert positions["left_shoulder_pitch_joint"] == -1.62772
    assert positions["left_shoulder_roll_joint"] == -0.5
    assert positions["left_shoulder_yaw_joint"] == 0.16625
    assert positions["left_elbow_pitch_joint"] == -2.52086
    assert positions["right_shoulder_pitch_joint"] == -1.2
    assert positions["right_elbow_pitch_joint"] == -2.4
    for side in ("left", "right"):
        assert f"{side}_wrist_yaw_joint" in positions
        assert f"{side}_wrist_pitch_joint" in positions
        assert f"{side}_wrist_roll_joint" in positions


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
