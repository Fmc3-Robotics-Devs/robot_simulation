"""Deterministic scenario contracts independent of simulator runtime."""

from franzi_sim.scenarios.box_transfer import (
    BOX_TRANSFER_CAMERA_REVIEW_POSE_RAD,
    BoxTransferConfig,
    BoxTransferStateMachine,
    RunScenarioRequest,
    TransferStage,
)

__all__ = [
    "BOX_TRANSFER_CAMERA_REVIEW_POSE_RAD",
    "BoxTransferConfig",
    "BoxTransferStateMachine",
    "RunScenarioRequest",
    "TransferStage",
]
