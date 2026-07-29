"""Minimal deterministic state contract for the mobile dual-arm box-transfer task."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class TransferStage(str, Enum):
    """Ordered stages of one box-transfer run."""

    SEARCH_TAG = "search_tag"
    APPROACH = "approach"
    ALIGN = "align"
    GRASP = "grasp"
    LIFT = "lift"
    TRANSPORT = "transport"
    PLACE = "place"
    RETREAT = "retreat"
    COMPLETE = "complete"
    FAILED = "failed"


@dataclass(frozen=True)
class BoxTransferConfig:
    """Static acceptance limits for one box-transfer scenario."""

    scenario_name: str = "warehouse_box_transfer"
    seed: int = 0
    max_retries: int = 1
    parking_tolerance_m: float = 0.03
    parking_tolerance_deg: float = 3.0


@dataclass(frozen=True)
class RunScenarioRequest:
    """Runtime-neutral input accepted by a future ROS 2 RunScenario action."""

    scenario: str
    seed: int = 0
    config_path: str | None = None


class BoxTransferStateMachine:
    """Advance the task only after each executor reports its stage complete."""

    _ORDER = (
        TransferStage.SEARCH_TAG,
        TransferStage.APPROACH,
        TransferStage.ALIGN,
        TransferStage.GRASP,
        TransferStage.LIFT,
        TransferStage.TRANSPORT,
        TransferStage.PLACE,
        TransferStage.RETREAT,
        TransferStage.COMPLETE,
    )

    def __init__(self, config: BoxTransferConfig) -> None:
        """Create a reset state machine for the supplied immutable configuration."""

        self.config = config
        self.stage = TransferStage.SEARCH_TAG
        self.retries = 0
        self.failure_reason: str | None = None

    @property
    def is_terminal(self) -> bool:
        """Return whether the run has reached a final success or failure state."""

        return self.stage in {TransferStage.COMPLETE, TransferStage.FAILED}

    def complete_stage(self) -> TransferStage:
        """Move to the next stage after its external executor confirms completion."""

        if self.is_terminal:
            return self.stage
        current_index = self._ORDER.index(self.stage)
        self.stage = self._ORDER[current_index + 1]
        return self.stage

    def fail_stage(self, reason: str) -> TransferStage:
        """Retry the current stage when allowed, otherwise terminate the run."""

        self.failure_reason = reason
        if self.retries < self.config.max_retries:
            self.retries += 1
            return self.stage
        # Failure is explicit so transport never continues after an unverified grasp or alignment.
        self.stage = TransferStage.FAILED
        return self.stage

    def reset(self) -> None:
        """Reset runtime state while preserving the scenario configuration."""

        self.stage = TransferStage.SEARCH_TAG
        self.retries = 0
        self.failure_reason = None
