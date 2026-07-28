"""Kinematic driver for the omnidirectional chassis.

The three wheels are steered independently, so the platform is holonomic: it
can translate in any direction while rotating. This module owns two things the
rest of the stack reads:

* the ``world -> moveit_root`` transform, which is what the planar virtual
  joint in the SRDF resolves to - move a robot without it and MoveIt keeps
  planning as if the chassis were still parked at the origin;
* the steering and wheel joint states, so the wheels in RViz actually point
  where the platform is going.

Motion is integrated, not simulated: no slip, no dynamics, no path planning
around obstacles. Driving is a straight line in the world plane with the
heading interpolated alongside it.
"""

import math
import threading
import time

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

# Steering joint origins in base_link, from the URDF.
WHEELS = {
    "left_front": (0.13, 0.213),
    "right_front": (0.13, -0.213),
    "rear": (-0.255, 0.0),
}
STEERING_JOINTS = {name: f"{name}_steering_joint" for name in WHEELS}
WHEEL_JOINTS = {name: f"{name}_wheel_joint" for name in WHEELS}


def normalise(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class MobileBase:
    def __init__(
        self,
        node,
        frame_id="world",
        child_frame="moveit_root",
        wheel_radius=0.0827,
        linear_speed=0.5,
        angular_speed=0.8,
        rate=50.0,
    ):
        self._node = node
        self._logger = node.get_logger()
        self._frame_id = frame_id
        self._child_frame = child_frame
        self._wheel_radius = wheel_radius
        self._linear_speed = linear_speed
        self._angular_speed = angular_speed
        self._period = 1.0 / rate

        self._lock = threading.Lock()
        self._pose = (0.0, 0.0, 0.0)
        self._steer = {name: 0.0 for name in WHEELS}
        self._spin = {name: 0.0 for name in WHEELS}

        self._tf = TransformBroadcaster(node)
        self._joints = node.create_publisher(JointState, "joint_states", 10)
        self._timer = node.create_timer(self._period, self._publish)
        self._publish()

    @property
    def pose(self):
        with self._lock:
            return self._pose

    def set_pose(self, x, y, yaw):
        with self._lock:
            self._pose = (x, y, yaw)
        self._publish()

    def drive_to(self, x, y, yaw, label=None):
        """Translate and rotate to a world pose, blocking until parked."""
        start_x, start_y, start_yaw = self.pose
        delta_yaw = normalise(yaw - start_yaw)
        distance = math.hypot(x - start_x, y - start_y)

        duration = max(
            distance / self._linear_speed,
            abs(delta_yaw) / self._angular_speed,
        )
        if duration < 1e-3:
            self.set_pose(x, y, yaw)
            return

        self._logger.info(
            f"{label or 'base'}: driving {distance:.2f} m / "
            f"{math.degrees(delta_yaw):+.0f} deg in {duration:.1f}s"
        )

        # World-frame velocity is constant over the move; the body-frame
        # velocity that the wheels see is not, because the platform rotates.
        velocity = ((x - start_x) / duration, (y - start_y) / duration)
        yaw_rate = delta_yaw / duration

        started = time.monotonic()
        previous = 0.0
        while True:
            elapsed = min(time.monotonic() - started, duration)
            alpha = elapsed / duration
            current_yaw = start_yaw + alpha * delta_yaw
            self._step(
                (
                    start_x + alpha * (x - start_x),
                    start_y + alpha * (y - start_y),
                    current_yaw,
                ),
                velocity,
                yaw_rate,
                current_yaw,
                elapsed - previous,
            )
            previous = elapsed
            if elapsed >= duration:
                break
            time.sleep(self._period)

        self.set_pose(x, y, yaw)

    def _step(self, pose, world_velocity, yaw_rate, yaw, dt):
        """Advance the pose and roll the wheels to match the motion."""
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        # World velocity expressed in the body frame.
        vx = world_velocity[0] * cos_yaw + world_velocity[1] * sin_yaw
        vy = -world_velocity[0] * sin_yaw + world_velocity[1] * cos_yaw

        with self._lock:
            self._pose = pose
            for name, (px, py) in WHEELS.items():
                wheel_vx = vx - yaw_rate * py
                wheel_vy = vy + yaw_rate * px
                speed = math.hypot(wheel_vx, wheel_vy)
                if speed < 1e-6:
                    continue

                angle = math.atan2(wheel_vy, wheel_vx)
                # Steering the short way round and driving backwards is the
                # same motion, and it keeps the joints away from their limits.
                if abs(normalise(angle - self._steer[name])) > math.pi / 2.0:
                    angle = normalise(angle + math.pi)
                    speed = -speed

                self._steer[name] = angle
                self._spin[name] += speed * dt / self._wheel_radius

        self._publish()

    def _publish(self):
        stamp = self._node.get_clock().now().to_msg()
        with self._lock:
            x, y, yaw = self._pose
            steer = dict(self._steer)
            spin = dict(self._spin)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._frame_id
        transform.child_frame_id = self._child_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self._tf.sendTransform(transform)

        state = JointState()
        state.header.stamp = stamp
        for name in WHEELS:
            state.name.append(STEERING_JOINTS[name])
            state.position.append(steer[name])
            state.name.append(WHEEL_JOINTS[name])
            state.position.append(normalise(spin[name]))
        self._joints.publish(state)
