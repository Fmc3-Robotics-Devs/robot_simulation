"""Kinematic driver for the omnidirectional chassis.

The three wheels are steered independently, so the platform is holonomic: it
can translate in any direction while rotating. This module owns two things the
rest of the stack reads:

* the ``odom -> moveit_root`` transform, which is what the planar virtual
  joint in the SRDF resolves to - move a robot without it and MoveIt keeps
  planning as if the chassis were still parked at the origin. On hardware this
  is wheel odometry's job, and Nav2 publishes ``map -> odom`` above it;
* the steering and wheel joint states, so the wheels in RViz actually point
  where the platform is going.

Motion is integrated, not simulated: no slip, no dynamics, no path planning
around obstacles. Driving is a straight line with the heading interpolated
alongside it. ``drive_to`` has the shape of a Nav2 ``NavigateToPose`` goal on
purpose - that is the intended replacement.
"""

import math
import random
import threading
import time

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
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
        frame_id="odom",
        child_frame="moveit_root",
        wheel_radius=0.0827,
        linear_speed=0.5,
        angular_speed=0.8,
        position_error=0.0,
        yaw_error=0.0,
        rate=50.0,
        callback_group=None,
    ):
        self._node = node
        self._logger = node.get_logger()
        self._frame_id = frame_id
        self._child_frame = child_frame
        self._wheel_radius = wheel_radius
        self._linear_speed = linear_speed
        self._angular_speed = angular_speed
        self._position_error = position_error
        self._yaw_error = yaw_error
        self._random = random.Random(0)
        self._period = 1.0 / rate

        self._lock = threading.Lock()
        self._pose = (0.0, 0.0, 0.0)
        self._steer = {name: 0.0 for name in WHEELS}
        self._spin = {name: 0.0 for name in WHEELS}

        self._tf = TransformBroadcaster(node)
        self._joints = node.create_publisher(JointState, "joint_states", 10)
        # The same pose as the transform, in a message a non-tf2 consumer can
        # subscribe to plainly - the Isaac mirror reads this to place the robot.
        self._pose_publisher = node.create_publisher(PoseStamped, "base/pose", 10)

        # The Nav2-facing half: velocity commands in, odometry out. Commands
        # are integrated by the publish timer; `drive_to` keeps priority, so a
        # late zero-twist from a finished navigation cannot fight a docking
        # correction.
        self._command = (0.0, 0.0, 0.0)
        self._command_time = 0.0
        self._velocity = (0.0, 0.0, 0.0)
        self._driving = False
        self._last_tick = time.monotonic()
        # Odometry is a heartbeat the whole navigation stack depends on. It
        # gets its own reentrant group so that no long-running skill callback
        # in the node's default group can starve it - a 27-second odom gap is
        # how "Initial robot pose is not available" happens mid-cycle.
        from rclpy.callback_groups import ReentrantCallbackGroup

        group = callback_group or ReentrantCallbackGroup()
        node.create_subscription(
            Twist, "cmd_vel", self._on_cmd_vel, 10, callback_group=group
        )
        self._odom_publisher = node.create_publisher(Odometry, "odom", 20)

        self._timer = node.create_timer(self._period, self._tick, callback_group=group)
        self._publish()

    @property
    def pose(self):
        with self._lock:
            return self._pose

    def set_pose(self, x, y, yaw):
        with self._lock:
            self._pose = (x, y, yaw)
        self._publish()

    # -- velocity interface (Nav2's side of the base) ----------------------

    def _on_cmd_vel(self, message):
        self._command = (message.linear.x, message.linear.y, message.angular.z)
        self._command_time = time.monotonic()

    def _tick(self):
        """Integrate any live velocity command, then publish state."""
        now = time.monotonic()
        dt = min(now - self._last_tick, 4.0 * self._period)
        self._last_tick = now

        vx, vy, wz = self._command
        fresh = (now - self._command_time) < 0.5
        moving = abs(vx) > 1e-4 or abs(vy) > 1e-4 or abs(wz) > 1e-4
        self._velocity = (vx, vy, wz) if (fresh and moving) else (0.0, 0.0, 0.0)
        if fresh and moving and not self._driving:
            x, y, yaw = self.pose
            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
            world_vx = cos_yaw * vx - sin_yaw * vy
            world_vy = sin_yaw * vx + cos_yaw * vy
            self._step(
                (x + world_vx * dt, y + world_vy * dt, normalise(yaw + wz * dt)),
                (world_vx, world_vy),
                wz,
                yaw,
                dt,
            )
        else:
            self._publish()

    def drive_to(self, x, y, yaw, label=None, exact=False, speed=None):
        """Drive to a goal and park near it. Returns where it actually stopped.

        Parking is deliberately imperfect. Navigation under SLAM lands within
        its goal tolerance, not on the goal, and that residual is the entire
        reason the station carries a tag: a base that always arrives exactly
        makes the tag pipeline look like it works while never asking it to do
        anything. Callers must use the returned pose, not the goal.

        ``exact`` skips the arrival scatter: it models the short, slow,
        visually-servoed corrections of precise docking, whose execution error
        is far below the tag observation noise that the loop already carries.
        ``speed`` caps the linear speed for those corrections.
        """
        self._driving = True
        try:
            return self._drive_to(x, y, yaw, label=label, exact=exact, speed=speed)
        finally:
            self._driving = False

    def _drive_to(self, x, y, yaw, label=None, exact=False, speed=None):
        start_x, start_y, start_yaw = self.pose
        if not exact:
            x, y, yaw = self._arrival(x, y, yaw)

        delta_yaw = normalise(yaw - start_yaw)
        distance = math.hypot(x - start_x, y - start_y)

        duration = max(
            distance / min(speed or self._linear_speed, self._linear_speed),
            abs(delta_yaw) / self._angular_speed,
        )
        if duration < 1e-3:
            self.set_pose(x, y, yaw)
            return self.pose

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
        return self.pose

    def _arrival(self, x, y, yaw):
        """Scatter the goal by the navigation stack's docking error."""
        if not self._position_error and not self._yaw_error:
            return x, y, yaw
        gauss = self._random.gauss
        parked = (
            x + gauss(0.0, self._position_error),
            y + gauss(0.0, self._position_error),
            normalise(yaw + gauss(0.0, self._yaw_error)),
        )
        self._logger.info(
            f"parked {math.hypot(parked[0] - x, parked[1] - y) * 1000:.0f} mm / "
            f"{math.degrees(normalise(parked[2] - yaw)):+.1f} deg off the goal"
        )
        return parked

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

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = transform.transform.rotation.z
        pose.pose.orientation.w = transform.transform.rotation.w
        self._pose_publisher.publish(pose)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._frame_id
        odom.child_frame_id = self._child_frame
        odom.pose.pose = pose.pose
        vx, vy, wz = self._velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self._odom_publisher.publish(odom)

        state = JointState()
        state.header.stamp = stamp
        for name in WHEELS:
            state.name.append(STEERING_JOINTS[name])
            state.position.append(steer[name])
            state.name.append(WHEEL_JOINTS[name])
            state.position.append(normalise(spin[name]))
        self._joints.publish(state)
