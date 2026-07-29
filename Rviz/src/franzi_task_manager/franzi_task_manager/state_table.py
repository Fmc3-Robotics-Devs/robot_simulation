"""The task state machine, as data.

Plan section 13 defines each state by entry condition, action, success
condition, timeout, retry budget and where to go when it fails. Encoding that
as a table rather than as control flow keeps the two things that change most
often - timeouts and recovery routing - out of the code, and makes the whole
sequence reviewable in one screen by someone who does not read Python.

Nothing here talks to ROS. It decides *what* should happen next; the node
decides how.
"""

from dataclasses import dataclass, field
from pathlib import Path

import yaml

TERMINAL = ("DONE", "FAULT")


@dataclass
class StateSpec:
    name: str
    # What to do on entry. Kind is one of skill, service, wait, none.
    action: dict = field(default_factory=dict)
    # Machine signals that must hold before the action may run at all.
    entry_conditions: list = field(default_factory=list)
    # Machine signals that must hold for the state to be considered done. Used
    # by `wait` states and as a post-check for the others.
    success_conditions: list = field(default_factory=list)
    # Robot signals to assert when the state begins / when it succeeds. This
    # is how `robot_clear` follows the arm through the cycle while the task
    # manager stays the only writer of the robot's signals.
    signals_on_entry: dict = field(default_factory=dict)
    signals_on_success: dict = field(default_factory=dict)
    timeout: float = 30.0
    max_retry: int = 0
    failure_transition: str = "FAULT"
    error_code: str = "UNSPECIFIED"
    next: str = ""

    @property
    def is_terminal(self):
        return self.name in TERMINAL


class StateTable:
    def __init__(self, states, initial):
        self._states = states
        self.initial = initial
        self._validate()

    @classmethod
    def load(cls, path):
        document = yaml.safe_load(Path(path).read_text())
        states = {}
        for name, spec in document["states"].items():
            spec = spec or {}
            states[name] = StateSpec(
                name=name,
                action=spec.get("action") or {},
                entry_conditions=spec.get("entry_conditions") or [],
                success_conditions=spec.get("success_conditions") or [],
                signals_on_entry=spec.get("signals_on_entry") or {},
                signals_on_success=spec.get("signals_on_success") or {},
                timeout=float(spec.get("timeout", 30.0)),
                max_retry=int(spec.get("max_retry", 0)),
                failure_transition=spec.get("failure_transition", "FAULT"),
                error_code=spec.get("error_code", "UNSPECIFIED"),
                next=spec.get("next", ""),
            )
        return cls(states, document.get("initial", "IDLE"))

    def __getitem__(self, name):
        if name not in self._states:
            raise KeyError(f"state '{name}' is not in the table")
        return self._states[name]

    def __contains__(self, name):
        return name in self._states

    @property
    def names(self):
        return list(self._states)

    def _validate(self):
        """Catch a broken table at load time rather than at 3 a.m.

        An unreachable failure_transition or a non-terminal state with nowhere
        to go both strand the machine mid-cycle, which is exactly when nobody
        wants to be reading YAML.
        """
        if self.initial not in self._states:
            raise ValueError(f"initial state '{self.initial}' is not defined")

        problems = []
        for spec in self._states.values():
            if spec.next and spec.next not in self._states:
                problems.append(f"{spec.name}.next -> unknown state '{spec.next}'")
            if spec.failure_transition not in self._states:
                problems.append(
                    f"{spec.name}.failure_transition -> unknown state "
                    f"'{spec.failure_transition}'"
                )
            if not spec.is_terminal and not spec.next:
                problems.append(f"{spec.name} has no next state and is not terminal")
        if problems:
            raise ValueError("state table is not runnable:\n  " + "\n  ".join(problems))
