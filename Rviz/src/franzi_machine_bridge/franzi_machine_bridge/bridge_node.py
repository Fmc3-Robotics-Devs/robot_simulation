#!/usr/bin/env python3
"""The engraving machine, as far as the rest of the system is concerned.

Publishes what the machine reports, accepts requests, and refuses the ones the
interlocks forbid. Every request that asks the machine to move something waits
for the confirming signal and fails on timeout (plan section 12.6) - success is
never inferred from having sent the request, because that is exactly how a robot
ends up reaching into a machine whose door did not actually open.

Swapping to hardware means passing a different MachineIO. Nothing in this node
knows whether it is talking to Modbus or to a simulation.
"""

import threading
import time

import rclpy
from franzi_engraving_interfaces.msg import MachineStatus, RobotSignals
from franzi_engraving_interfaces.srv import (
    ResetFault,
    SetClamp,
    SetDoor,
    StartMachining,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

from .interlocks import may_move_door, may_start_machining
from .machine_io import ROBOT_SIGNALS, SimulatedMachine, SimulatedTimings

# Plan section 12.6.
DEFAULT_TIMEOUTS = {
    "door_open": 10.0,
    "door_close": 10.0,
    "clamp_open": 5.0,
    "clamp_close": 5.0,
    "material": 3.0,
}


class MachineBridge(Node):
    def __init__(self):
        super().__init__("machine_bridge")

        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("watchdog_timeout", 1.0)
        for name, value in DEFAULT_TIMEOUTS.items():
            self.declare_parameter(f"timeout.{name}", value)
        self.declare_parameter("simulated.door_seconds", 2.5)
        self.declare_parameter("simulated.clamp_seconds", 1.2)
        self.declare_parameter("simulated.spindle_spin_down_seconds", 2.0)
        self.declare_parameter("simulated.machining_seconds", 8.0)

        self._io = SimulatedMachine(
            SimulatedTimings(
                door=self.get("simulated.door_seconds"),
                clamp=self.get("simulated.clamp_seconds"),
                spindle_spin_down=self.get("simulated.spindle_spin_down_seconds"),
                machining=self.get("simulated.machining_seconds"),
            ),
            logger=self.get_logger(),
        )
        self._lock = threading.Lock()
        self._robot = RobotSignals()
        self._status = MachineStatus()
        self._last_read = 0.0

        group = ReentrantCallbackGroup()
        self._status_publisher = self.create_publisher(MachineStatus, "machine/status", 10)
        self.create_subscription(
            RobotSignals, "robot/signals", self._on_robot_signals, 10, callback_group=group
        )
        self.create_timer(
            1.0 / self.get("publish_rate"), self._publish_status, callback_group=group
        )

        # Simulation seam: the skill layer reports having released into, or
        # lifted out of, the clamp. On hardware nothing publishes this topic -
        # the machine's own presence sensor reports through MachineIO - and a
        # backend without the hook ignores it rather than believing a robot's
        # claim about a sensor it does not own.
        self.create_subscription(
            Bool,
            "machine/sim/material_present",
            lambda message: self.set_material_present(message.data),
            1,
            callback_group=group,
        )

        self.create_service(SetDoor, "machine/set_door", self._set_door, callback_group=group)
        self.create_service(SetClamp, "machine/set_clamp", self._set_clamp, callback_group=group)
        self.create_service(
            StartMachining, "machine/start_machining", self._start, callback_group=group
        )
        self.create_service(ResetFault, "machine/reset_fault", self._reset, callback_group=group)

        self.get_logger().info("machine bridge up (simulated backend)")

    def get(self, name):
        return self.get_parameter(name).value

    # -- status ------------------------------------------------------------

    def _publish_status(self):
        try:
            state = self._io.read()
            self._last_read = time.monotonic()
            link_ok = True
        except Exception as error:  # noqa: BLE001 - any transport fault is a link fault
            self.get_logger().warning(f"machine read failed: {error}")
            link_ok = (time.monotonic() - self._last_read) < self.get("watchdog_timeout")
            state = None

        status = MachineStatus()
        status.stamp = self.get_clock().now().to_msg()
        status.link_ok = link_ok
        if state is not None:
            for name, value in state.as_dict().items():
                setattr(status, name, value)

        with self._lock:
            self._status = status
        self._status_publisher.publish(status)

    def _snapshot(self):
        with self._lock:
            return self._status, self._robot

    def _on_robot_signals(self, message):
        with self._lock:
            self._robot = message
        self._io.write({name: getattr(message, name) for name in ROBOT_SIGNALS})

    # -- waiting -----------------------------------------------------------

    def _wait_for(self, predicate, timeout, what):
        """Wait for a machine signal to confirm an actuation actually happened."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status, _ = self._snapshot()
            if not status.link_ok:
                return False, "LINK_LOST"
            if status.emergency_stop:
                return False, "EMERGENCY_STOP"
            if status.alarm:
                return False, "MACHINE_ALARM"
            if predicate(status):
                return True, ""
            time.sleep(0.05)
        self.get_logger().error(f"timed out waiting for {what} after {timeout:.1f}s")
        return False, "TIMEOUT"

    # -- services ----------------------------------------------------------

    def _set_door(self, request, response):
        status, _ = self._snapshot()
        verdict = may_move_door(status)
        if not verdict:
            response.success = False
            response.error_code = f"BLOCKED_{verdict.blocked_by.upper()}"
            self.get_logger().warning(f"door request refused: {verdict.blocked_by}")
            return response

        self._io.command("close_door" if request.close else "open_door")
        timeout = self.get("timeout.door_close" if request.close else "timeout.door_open")
        confirmed, error = self._wait_for(
            (lambda s: s.door_closed) if request.close else (lambda s: s.door_open),
            timeout,
            "door closed" if request.close else "door open",
        )
        response.success = confirmed
        response.error_code = error
        return response

    def _set_clamp(self, request, response):
        status, _ = self._snapshot()
        if not status.link_ok:
            response.success, response.error_code = False, "LINK_LOST"
            return response

        self._io.command("close_clamp" if request.close else "open_clamp")
        timeout = self.get("timeout.clamp_close" if request.close else "timeout.clamp_open")
        confirmed, error = self._wait_for(
            (lambda s: s.clamp_closed) if request.close else (lambda s: s.clamp_open),
            timeout,
            "clamp closed" if request.close else "clamp open",
        )
        response.success = confirmed
        response.error_code = error
        return response

    def _start(self, _request, response):
        status, robot = self._snapshot()
        verdict = may_start_machining(status, robot)
        if not verdict:
            response.success = False
            response.error_code = "INTERLOCK"
            response.blocked_by = verdict.blocked_by
            self.get_logger().warning(f"start refused: {verdict.blocked_by}")
            return response

        self._io.command("start_machining")
        confirmed, error = self._wait_for(lambda s: s.machining, 5.0, "machining to begin")
        response.success = confirmed
        response.error_code = error
        return response

    def _reset(self, _request, response):
        self._io.command("reset")
        response.success = True
        return response

    # -- simulation hook ---------------------------------------------------

    def set_material_present(self, present):
        """The gripper released into, or lifted out of, the clamp."""
        if hasattr(self._io, "set_material_present"):
            self._io.set_material_present(present)
        else:
            self.get_logger().warning(
                "material_present report ignored: this machine has its own sensor"
            )


def main():
    rclpy.init()
    node = MachineBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
