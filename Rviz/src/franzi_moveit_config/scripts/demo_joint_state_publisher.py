#!/usr/bin/env python3
"""A state publisher and virtual trajectory controller for the RViz demo."""

import threading
import time
from math import ceil
from pathlib import Path
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


CONTROLLERS = {
    "body_controller": (
        "calf_pitch_joint",
        "thigh_pitch_joint",
        "waist_pitch_joint",
        "waist_yaw_joint",
    ),
    "head_controller": ("head_yaw_joint", "head_pitch_joint"),
    "left_arm_controller": (
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_pitch_joint",
        "left_wrist_yaw_joint",
        "left_wrist_pitch_joint",
        "left_wrist_roll_joint",
    ),
    "right_arm_controller": (
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_pitch_joint",
        "right_wrist_yaw_joint",
        "right_wrist_pitch_joint",
        "right_wrist_roll_joint",
    ),
    "left_gripper_controller": ("leftfinger1_joint", "leftfinger2_joint"),
    "right_gripper_controller": ("rightfinger1_joint", "rightfinger2_joint"),
}


def load_initial_joint_state():
    """Return movable, non-mimic joints and a valid zero-biased position."""
    urdf_path = (
        Path(get_package_share_directory("franzi_description"))
        / "urdf"
        / "wheel_robot_4.0.urdf"
    )
    root = ElementTree.parse(urdf_path).getroot()

    names = []
    positions = []
    for joint in root.findall("joint"):
        if joint.get("type") == "fixed" or joint.find("mimic") is not None:
            continue

        limit = joint.find("limit")
        lower = float(limit.get("lower", "0.0")) if limit is not None else 0.0
        upper = float(limit.get("upper", "0.0")) if limit is not None else 0.0
        names.append(joint.get("name"))
        positions.append(min(max(0.0, lower), upper))

    return names, positions


def duration_seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


class DemoRobotDriver(Node):
    def __init__(self):
        super().__init__("demo_robot_driver")
        names, positions = load_initial_joint_state()
        self._names = names
        self._state = dict(zip(names, positions))
        self._state_lock = threading.Lock()
        self._publisher = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(0.05, self.publish_state)

        self._action_group = ReentrantCallbackGroup()
        self._action_servers = [
            ActionServer(
                self,
                FollowJointTrajectory,
                f"{controller}/follow_joint_trajectory",
                execute_callback=self.execute_trajectory,
                goal_callback=self.accept_goal,
                cancel_callback=self.cancel_goal,
                callback_group=self._action_group,
            )
            for controller in CONTROLLERS
        ]
        self.get_logger().info(
            f"Publishing initial states for {len(self._names)} movable joints and "
            f"serving {len(self._action_servers)} virtual trajectory controllers."
        )

    def publish_state(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self._names
        with self._state_lock:
            message.position = [self._state[name] for name in self._names]
        self._publisher.publish(message)

    def accept_goal(self, goal):
        names = list(goal.trajectory.joint_names)
        if not names or not goal.trajectory.points:
            self.get_logger().warning("Rejecting an empty trajectory goal.")
            return GoalResponse.REJECT
        if any(name not in self._state for name in names):
            self.get_logger().warning("Rejecting a trajectory with unknown joints.")
            return GoalResponse.REJECT
        if any(len(point.positions) != len(names) for point in goal.trajectory.points):
            self.get_logger().warning("Rejecting a trajectory with invalid positions.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def cancel_goal(_goal_handle):
        return CancelResponse.ACCEPT

    def execute_trajectory(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        names = list(trajectory.joint_names)
        with self._state_lock:
            start_positions = [self._state[name] for name in names]

        previous_positions = start_positions
        previous_time = 0.0
        started_at = time.monotonic()
        for point in trajectory.points:
            target_positions = list(point.positions)
            target_time = max(previous_time, duration_seconds(point.time_from_start))
            interval = target_time - previous_time
            steps = max(1, ceil(interval / 0.02))

            for step in range(1, steps + 1):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result()

                alpha = step / steps
                interpolated = [
                    start + alpha * (target - start)
                    for start, target in zip(previous_positions, target_positions)
                ]
                with self._state_lock:
                    self._state.update(zip(names, interpolated))
                self.publish_state()

                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = names
                feedback.desired.positions = interpolated
                feedback.actual.positions = interpolated
                feedback.error.positions = [0.0] * len(names)
                goal_handle.publish_feedback(feedback)

                remaining = started_at + previous_time + alpha * interval - time.monotonic()
                if remaining > 0.0:
                    time.sleep(remaining)

            previous_positions = target_positions
            previous_time = target_time

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def destroy_node(self):
        for action_server in self._action_servers:
            action_server.destroy()
        super().destroy_node()


def main():
    rclpy.init()
    node = DemoRobotDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
