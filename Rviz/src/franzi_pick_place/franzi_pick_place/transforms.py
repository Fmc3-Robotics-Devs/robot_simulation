"""4x4 homogeneous transform helpers.

Pose maths gets done here rather than inline, because the tag pipeline chains
five frames together and sign errors in that chain are exactly the kind of bug
that only shows up as a gripper 2 cm off the part.
"""

import math

import numpy as np
from geometry_msgs.msg import Pose, Quaternion


def matrix_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def matrix_from_quaternion(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def quaternion_from_matrix(rotation):
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2][1] - rotation[1][2]) / scale
        y = (rotation[0][2] - rotation[2][0]) / scale
        z = (rotation[1][0] - rotation[0][1]) / scale
    elif rotation[0][0] > rotation[1][1] and rotation[0][0] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]) * 2.0
        w = (rotation[2][1] - rotation[1][2]) / scale
        x = 0.25 * scale
        y = (rotation[0][1] + rotation[1][0]) / scale
        z = (rotation[0][2] + rotation[2][0]) / scale
    elif rotation[1][1] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]) * 2.0
        w = (rotation[0][2] - rotation[2][0]) / scale
        x = (rotation[0][1] + rotation[1][0]) / scale
        y = 0.25 * scale
        z = (rotation[1][2] + rotation[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]) * 2.0
        w = (rotation[1][0] - rotation[0][1]) / scale
        x = (rotation[0][2] + rotation[2][0]) / scale
        y = (rotation[1][2] + rotation[2][1]) / scale
        z = 0.25 * scale
    return Quaternion(x=x, y=y, z=z, w=w)


def transform(translation=(0.0, 0.0, 0.0), rotation=None):
    matrix = np.eye(4)
    if rotation is not None:
        matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def from_rpy(translation, rpy):
    return transform(translation, matrix_from_rpy(*rpy))


def from_pose(pose: Pose):
    return transform(
        (pose.position.x, pose.position.y, pose.position.z),
        matrix_from_quaternion(pose.orientation),
    )


def to_pose(matrix) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in matrix[:3, 3])
    pose.orientation = quaternion_from_matrix(matrix[:3, :3])
    return pose


def from_transform_msg(msg):
    """Build a matrix from a geometry_msgs/Transform."""
    return transform(
        (msg.translation.x, msg.translation.y, msg.translation.z),
        matrix_from_quaternion(msg.rotation),
    )


def invert(matrix):
    rotation = matrix[:3, :3]
    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ matrix[:3, 3]
    return inverse


def yaw_of(matrix):
    return math.atan2(matrix[1][0], matrix[0][0])
