#!/usr/bin/env python3
"""Runs the engraving cycle described by the state table.

The orchestration lives here; what each state means lives in `states.yaml`.
This node's job is narrow on purpose: check the entry conditions, dispatch the
action, enforce the timeout, count the retries, and route the failure. It knows
nothing about arms, tags or clamps.

The one behaviour worth stating outright: a state never succeeds by default. It
succeeds when its action reported success *and* its success conditions hold. A
timeout is a failure even if the thing being waited on later turns up.
"""

import threading
import time

import rclpy
from franzi_engraving_interfaces.msg import MachineStatus, RobotSignals, TaskState
from franzi_engraving_interfaces.srv import ClearTask, SetClamp, SetDoor, StartMachining
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .skill_client import SkillClient
from .state_table import StateTable


def evaluate(condition: str, status) -> bool:
    """Evaluate one condition string against a MachineStatus."""
    negated = condition.startswith("not ")
    field = condition[4:] if negated else condition
    if not hasattr(status, field):
        raise ValueError(f"'{field}' is not a machine signal")
    value = bool(getattr(status, field))
    return (not value) if negated else value


class TaskManager(Node):
    def __init__(self):
        super().__init__("task_manager")
        self.declare_parameter("state_table", "")
        self.declare_parameter("machine_service_timeout", 20.0)
        self.declare_parameter("autostart", True)
        self.declare_parameter("cycles", 1)

        path = self.get_parameter("state_table").value
        if not path:
            from ament_index_python.packages import get_package_share_directory

            path = f"{get_package_share_directory('franzi_task_manager')}/config/states.yaml"
        self._table = StateTable.load(path)
        self.get_logger().info(f"state table: {len(self._table.names)} states from {path}")

        group = ReentrantCallbackGroup()
        self._status = MachineStatus()
        self._status_seen = False
        self._lock = threading.Lock()
        self._abort = threading.Event()
        # The full robot-signal set as last asserted; see assert_signals.
        self._asserted = {
            "load_request": False,
            "unload_request": False,
            "start_request": False,
            "robot_clear": True,
            "robot_busy": False,
            "robot_fault": False,
            "reset_request": False,
        }

        self.create_subscription(
            MachineStatus, "machine/status", self._on_status, 10, callback_group=group
        )
        self._signals = self.create_publisher(RobotSignals, "robot/signals", 10)
        self._state_publisher = self.create_publisher(TaskState, "task/state", 10)
        self.create_service(ClearTask, "task/clear", self._on_clear, callback_group=group)

        self._machine = {
            "set_door": self.create_client(SetDoor, "machine/set_door", callback_group=group),
            "set_clamp": self.create_client(SetClamp, "machine/set_clamp", callback_group=group),
            "start_machining": self.create_client(
                StartMachining, "machine/start_machining", callback_group=group
            ),
        }
        self._skills = SkillClient(self, group)

        self._state = self._table.initial
        self._previous = ""
        self._entered = time.monotonic()
        self._retries = 0
        self._error = ""
        self._detail = ""
        self.create_timer(0.2, self._publish_state, callback_group=group)

    # -- plumbing ----------------------------------------------------------

    def _on_status(self, message):
        with self._lock:
            self._status = message
            self._status_seen = True

    def status(self):
        with self._lock:
            return self._status

    def _on_clear(self, _request, response):
        self._abort.set()
        response.success = True
        return response

    def _publish_state(self):
        message = TaskState()
        message.stamp = self.get_clock().now().to_msg()
        message.state = self._state
        message.previous_state = self._previous
        message.retry_count = self._retries
        message.seconds_in_state = time.monotonic() - self._entered
        message.error_code = self._error
        message.detail = self._detail
        self._state_publisher.publish(message)

    def assert_signals(self, **values):
        """Tell the machine what the robot is doing (plan section 12.3).

        Signals not named keep their previous value: the message carries the
        whole set, so publishing only the change would silently drop
        `robot_clear` back to False every time `robot_busy` is asserted.
        This node is the only writer of the robot's signals for that reason.
        """
        with self._lock:
            self._asserted.update({name: bool(value) for name, value in values.items()})
            message = RobotSignals()
            message.stamp = self.get_clock().now().to_msg()
            for name, value in self._asserted.items():
                setattr(message, name, value)
        self._signals.publish(message)

    # -- conditions --------------------------------------------------------

    def conditions_hold(self, conditions):
        status = self.status()
        for condition in conditions:
            if not evaluate(condition, status):
                return False, condition
        return True, ""

    def wait_for_conditions(self, conditions, timeout):
        deadline = time.monotonic() + timeout
        blocker = ""
        while time.monotonic() < deadline and not self._abort.is_set():
            held, blocker = self.conditions_hold(conditions)
            if held:
                return True, ""
            time.sleep(0.05)
        return False, blocker

    # -- actions -----------------------------------------------------------

    def _run_service(self, action, timeout):
        name = action["service"]
        client = self._machine[name]
        if not client.wait_for_service(timeout_sec=5.0):
            return False, "SERVICE_UNAVAILABLE"

        request = client.srv_type.Request()
        if "close" in action:
            request.close = bool(action["close"])
        future = client.call_async(request)

        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return False, "TIMEOUT"

        response = future.result()
        if response is None or not response.success:
            code = getattr(response, "error_code", "") or "REFUSED"
            blocked = getattr(response, "blocked_by", "")
            return False, f"{code}:{blocked}" if blocked else code
        return True, ""

    def run_action(self, spec):
        action = spec.action or {}
        kind = action.get("kind", "none")
        if kind == "none":
            return True, ""
        if kind == "wait":
            held, blocker = self.wait_for_conditions(spec.success_conditions, spec.timeout)
            return held, "" if held else f"BLOCKED:{blocker}"
        if kind == "service":
            return self._run_service(action, spec.timeout)
        if kind == "skill":
            return self._skills.run(action, spec.timeout, self._abort)
        return False, f"UNKNOWN_ACTION:{kind}"

    # -- the cycle ---------------------------------------------------------

    def enter(self, name):
        self._previous, self._state = self._state, name
        self._entered = time.monotonic()
        self._retries = 0
        self.get_logger().info(f"--> {name}")

    def run(self):
        deadline = time.monotonic() + 30.0
        while not self._status_seen and time.monotonic() < deadline:
            time.sleep(0.1)
        if not self._status_seen:
            self.get_logger().error("no machine status; is the bridge running?")
            self.enter("FAULT")
            return 1

        cycles = max(1, self.get_parameter("cycles").value)
        for cycle in range(cycles):
            self.get_logger().info(f"=== cycle {cycle + 1}/{cycles} ===")
            self.enter(self._table.initial)
            if not self._run_once():
                return 1
        return 0

    def _run_once(self):
        while not self._abort.is_set():
            spec = self._table[self._state]
            if spec.is_terminal:
                self.assert_signals(robot_busy=False, **spec.signals_on_entry)
                return spec.name == "DONE"

            held, blocker = self.conditions_hold(spec.entry_conditions)
            if not held:
                self._fail(spec, f"ENTRY:{blocker}")
                continue

            self.assert_signals(robot_busy=True, **spec.signals_on_entry)
            succeeded, detail = self.run_action(spec)

            if succeeded and spec.success_conditions and spec.action.get("kind") != "wait":
                succeeded, blocker = self.wait_for_conditions(
                    spec.success_conditions, min(spec.timeout, 10.0)
                )
                detail = "" if succeeded else f"POST:{blocker}"

            if succeeded:
                self._error = self._detail = ""
                self.assert_signals(**spec.signals_on_success)
                self.enter(spec.next)
            else:
                self._fail(spec, detail)

        self.get_logger().warning("task cleared")
        return False

    def _fail(self, spec, detail):
        if self._retries < spec.max_retry:
            self._retries += 1
            self.get_logger().warning(
                f"{spec.name} failed ({detail}); retry {self._retries}/{spec.max_retry}"
            )
            self._entered = time.monotonic()
            return

        self._error = spec.error_code
        self._detail = detail
        self.get_logger().error(
            f"{spec.name} failed ({spec.error_code}: {detail}) -> {spec.failure_transition}"
        )
        self.assert_signals(robot_fault=True, robot_busy=False)
        self.enter(spec.failure_transition)


def main():
    rclpy.init()
    node = TaskManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    code = 0
    try:
        if node.get_parameter("autostart").value:
            code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
