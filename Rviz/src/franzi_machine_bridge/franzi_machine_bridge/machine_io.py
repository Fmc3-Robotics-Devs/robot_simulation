"""The seam between the interlock logic and whatever the machine actually is.

Everything safety-relevant - what may start, what may enter, what counts as a
timeout - lives in the bridge node and is hardware independent. This module is
only the transport: on hardware it becomes Modbus coils or digital inputs, and
nothing above it changes.

The one rule an implementation must respect: never invent a signal. If the
machine cannot report whether its clamp is closed, the field stays False and
the interlock refuses to proceed. A guessed signal is worse than a missing one,
because the guess is what gets believed at three in the morning.
"""

import threading
import time
from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass

# Signals the machine reports (plan section 12.2).
MACHINE_SIGNALS = (
    "machine_ready",
    "door_open",
    "door_closed",
    "clamp_open",
    "clamp_closed",
    "material_detected",
    "machining",
    "finished",
    "spindle_stopped",
    "alarm",
    "emergency_stop",
)

# Signals the robot asserts back (plan section 12.3).
ROBOT_SIGNALS = (
    "load_request",
    "unload_request",
    "start_request",
    "robot_clear",
    "robot_busy",
    "robot_fault",
    "reset_request",
)


@dataclass
class MachineState:
    machine_ready: bool = False
    door_open: bool = False
    door_closed: bool = True
    clamp_open: bool = False
    clamp_closed: bool = True
    material_detected: bool = False
    machining: bool = False
    finished: bool = False
    spindle_stopped: bool = True
    alarm: bool = False
    emergency_stop: bool = False

    def as_dict(self):
        return asdict(self)


class MachineIO(ABC):
    """Read the machine's signals, write the robot's."""

    @abstractmethod
    def read(self) -> MachineState:
        """Current machine signals. Raises if the link is down."""

    @abstractmethod
    def write(self, signals: dict) -> None:
        """Assert the robot-side signals (plan section 12.3)."""

    @abstractmethod
    def command(self, name: str) -> None:
        """Request an actuation: open_door, close_door, open_clamp,
        close_clamp, start_machining, reset."""


@dataclass
class SimulatedTimings:
    """How long the simulated machine takes to do things.

    Deliberately not instant. A machine that responds within one control cycle
    hides every missing wait in the caller, and those only show up on hardware.
    """

    door: float = 2.5
    clamp: float = 1.2
    spindle_spin_down: float = 2.0
    machining: float = 8.0


class SimulatedMachine(MachineIO):
    """A machine that behaves like one: things take time and can be refused.

    Actuation is modelled as a transition with a duration rather than a flag
    flip, so both the door-open and door-closed signals are false while it
    moves - which is what a real pair of limit switches reports, and what the
    caller has to be able to cope with.
    """

    def __init__(self, timings: SimulatedTimings = None, logger=None):
        self._timings = timings or SimulatedTimings()
        self._logger = logger
        self._lock = threading.Lock()
        self._state = MachineState(machine_ready=True)
        self._robot = {name: False for name in ROBOT_SIGNALS}
        self._transitions = []

    # -- MachineIO ---------------------------------------------------------

    def read(self) -> MachineState:
        with self._lock:
            self._advance()
            return MachineState(**self._state.as_dict())

    def write(self, signals: dict) -> None:
        with self._lock:
            self._robot.update(
                {name: bool(value) for name, value in signals.items() if name in self._robot}
            )

    def command(self, name: str) -> None:
        with self._lock:
            self._advance()
            handler = getattr(self, f"_do_{name}", None)
            if handler is None:
                raise ValueError(f"unknown machine command '{name}'")
            handler()

    # -- simulated mechanics ----------------------------------------------

    def _schedule(self, delay, apply):
        self._transitions.append((time.monotonic() + delay, apply))

    def _advance(self):
        now = time.monotonic()
        due = [entry for entry in self._transitions if entry[0] <= now]
        self._transitions = [entry for entry in self._transitions if entry[0] > now]
        for _, apply in due:
            apply()

    def _do_open_door(self):
        if self._state.door_open:
            return
        self._state.door_closed = False
        self._schedule(self._timings.door, lambda: setattr(self._state, "door_open", True))

    def _do_close_door(self):
        if self._state.door_closed:
            return
        self._state.door_open = False
        self._schedule(self._timings.door, lambda: setattr(self._state, "door_closed", True))

    def _do_open_clamp(self):
        if self._state.clamp_open:
            return
        self._state.clamp_closed = False
        self._schedule(self._timings.clamp, lambda: setattr(self._state, "clamp_open", True))

    def _do_close_clamp(self):
        if self._state.clamp_closed:
            return
        self._state.clamp_open = False
        self._schedule(self._timings.clamp, lambda: setattr(self._state, "clamp_closed", True))

    def _do_start_machining(self):
        self._state.machining = True
        self._state.finished = False
        self._state.spindle_stopped = False

        def complete():
            self._state.machining = False
            self._state.finished = True
            self._schedule(
                self._timings.spindle_spin_down,
                lambda: setattr(self._state, "spindle_stopped", True),
            )

        self._schedule(self._timings.machining, complete)

    def _do_reset(self):
        self._state.alarm = False
        self._state.finished = False
        self._transitions.clear()

    # -- simulation-only hooks --------------------------------------------

    def set_material_present(self, present: bool) -> None:
        """Called when the robot's gripper releases into or lifts out of the
        clamp. On hardware this is the machine's own presence sensor."""
        with self._lock:
            self._state.material_detected = bool(present)

    def raise_alarm(self, reason: str = "") -> None:
        with self._lock:
            self._state.alarm = True
            self._state.machining = False
            if self._logger:
                self._logger.error(f"machine alarm: {reason}")

    @property
    def robot_signals(self) -> dict:
        with self._lock:
            return dict(self._robot)
