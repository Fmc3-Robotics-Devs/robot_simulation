"""Tests for the runtime-neutral box-transfer state contract."""

from franzi_sim.scenarios.box_transfer import BoxTransferConfig, BoxTransferStateMachine, TransferStage


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
