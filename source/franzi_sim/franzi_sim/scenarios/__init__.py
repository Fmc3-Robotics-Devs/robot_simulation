"""Deterministic scenario contracts independent of simulator runtime."""

from franzi_sim.scenarios.box_transfer import (
    BOX_TRANSFER_CAMERA_REVIEW_POSE_RAD,
    BoxTransferConfig,
    BoxTransferStateMachine,
    RunScenarioRequest,
    TransferStage,
)
from franzi_sim.scenarios.box_transfer_demo import (
    BOX_INITIAL_POSITION_M,
    BOX_TARGET_POSITION_M,
    DEMO_SEGMENTS,
    DemoSample,
    demo_duration_s,
    sample_demo,
)

__all__ = [
    "BOX_TRANSFER_CAMERA_REVIEW_POSE_RAD",
    "BoxTransferConfig",
    "BoxTransferStateMachine",
    "RunScenarioRequest",
    "TransferStage",
    "BOX_INITIAL_POSITION_M",
    "BOX_TARGET_POSITION_M",
    "DEMO_SEGMENTS",
    "DemoSample",
    "demo_duration_s",
    "sample_demo",
]
