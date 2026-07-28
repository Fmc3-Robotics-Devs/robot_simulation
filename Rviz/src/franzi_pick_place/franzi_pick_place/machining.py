"""The machining station the workpiece is handed to.

There is no process model here - the station only owns the handshake the robot
has to respect: once the part is in the pocket the cycle runs, and the part may
only be taken back out when the station reports done. The status light makes
that handshake visible in RViz.
"""

import threading
import time

from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import make_pose

IDLE = "idle"
BUSY = "machining"
DONE = "done"

LIGHT_COLORS = {
    IDLE: (0.45, 0.45, 0.45, 1.0),
    BUSY: (0.95, 0.25, 0.10, 1.0),
    DONE: (0.15, 0.85, 0.25, 1.0),
}


class MachiningStation:
    def __init__(self, node, layout, duration, wait_for_trigger):
        self._node = node
        self._logger = node.get_logger()
        self._layout = layout
        self._duration = duration
        self._wait_for_trigger = wait_for_trigger

        self._state = IDLE
        self._lock = threading.Lock()
        self._finished = threading.Event()
        self._blink = False

        self._markers = node.create_publisher(MarkerArray, "machining_station/markers", 1)
        self._service = node.create_service(
            Trigger, "machining_station/complete", self._on_complete
        )
        self._timer = node.create_timer(0.25, self._publish)

    @property
    def state(self):
        with self._lock:
            return self._state

    def _set_state(self, state):
        with self._lock:
            self._state = state
        self._publish()

    def _on_complete(self, _request, response):
        self._finished.set()
        response.success = True
        response.message = "machining cycle marked complete"
        return response

    def run_cycle(self):
        """Block until the machining cycle is over."""
        self._finished.clear()
        self._set_state(BUSY)

        if self._wait_for_trigger:
            self._logger.info(
                "machining started, waiting for /machining_station/complete"
            )
            self._finished.wait()
        else:
            self._logger.info(f"machining started, cycle time {self._duration:.1f}s")
            deadline = time.monotonic() + self._duration
            while not self._finished.is_set() and time.monotonic() < deadline:
                time.sleep(0.1)

        self._set_state(DONE)
        self._logger.info("machining complete, part released for pick-up")

    def reset(self):
        self._set_state(IDLE)

    def _publish(self):
        with self._lock:
            state = self._state
        self._blink = not self._blink

        red, green, blue, alpha = LIGHT_COLORS[state]
        if state == BUSY and self._blink:
            red, green, blue = red * 0.35, green * 0.35, blue * 0.35

        x, y = self._layout.pocket_xy
        base_z = self._layout.table_top_z

        light = Marker()
        light.header.frame_id = self._layout.frame_id
        light.header.stamp = self._node.get_clock().now().to_msg()
        light.ns = "machining_station"
        light.id = 0
        light.type = Marker.SPHERE
        light.action = Marker.ADD
        light.pose = make_pose((x, y, base_z + 0.30))
        light.scale.x = light.scale.y = light.scale.z = 0.07
        light.color.r, light.color.g, light.color.b, light.color.a = red, green, blue, alpha

        label = Marker()
        label.header = light.header
        label.ns = "machining_station"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose = make_pose((x, y, base_z + 0.40))
        label.scale.z = 0.05
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = state

        self._markers.publish(MarkerArray(markers=[light, label]))
