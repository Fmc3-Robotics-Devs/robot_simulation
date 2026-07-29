"""The state table has to be runnable before anything runs it.

A table that routes a failure to a state that does not exist, or leaves a
non-terminal state with nowhere to go, strands the robot mid-cycle. Catching
that at load time is worth more than any runtime handling.
"""

import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from franzi_task_manager.state_table import StateTable  # noqa: E402

TABLE = ROOT / "config" / "states.yaml"


def test_the_shipped_table_loads():
    table = StateTable.load(TABLE)
    assert table.initial == "IDLE"
    assert "FAULT" in table.names and "DONE" in table.names


def test_every_state_is_reachable_from_the_initial_one():
    table = StateTable.load(TABLE)
    seen, frontier = set(), [table.initial]
    while frontier:
        name = frontier.pop()
        if name in seen:
            continue
        seen.add(name)
        spec = table[name]
        frontier += [n for n in (spec.next, spec.failure_transition) if n]
    assert set(table.names) - seen == set()


def test_a_dangling_next_is_refused(tmp_path):
    path = tmp_path / "broken.yaml"
    path.write_text(
        yaml.safe_dump(
            {"initial": "A", "states": {"A": {"next": "NOWHERE"}, "DONE": {}, "FAULT": {}}}
        )
    )
    with pytest.raises(ValueError, match="unknown state 'NOWHERE'"):
        StateTable.load(path)


def test_a_state_with_no_exit_is_refused(tmp_path):
    path = tmp_path / "stuck.yaml"
    path.write_text(
        yaml.safe_dump({"initial": "A", "states": {"A": {}, "DONE": {}, "FAULT": {}}})
    )
    with pytest.raises(ValueError, match="no next state"):
        StateTable.load(path)


def test_conditions_reference_real_machine_signals():
    # A typo in a condition would otherwise only surface as a state that never
    # passes, minutes into a cycle.
    from franzi_machine_bridge.machine_io import MACHINE_SIGNALS

    allowed = set(MACHINE_SIGNALS) | {"link_ok"}
    document = yaml.safe_load(TABLE.read_text())
    for name, spec in document["states"].items():
        for condition in (spec or {}).get("entry_conditions", []) + (
            (spec or {}).get("success_conditions", [])
        ):
            field = condition[4:] if condition.startswith("not ") else condition
            assert field in allowed, f"{name} refers to unknown signal '{field}'"
