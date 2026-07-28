"""Small pose helpers.

Everything in this package works in the MoveIt planning frame (``world``),
which for this robot coincides with ``base_link``.
"""

import math

from geometry_msgs.msg import Point, Pose, Quaternion


def quaternion_from_rpy(roll, pitch, yaw):
    """Return a Quaternion from intrinsic X-Y-Z (roll-pitch-yaw) angles."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def rotate_vector(quaternion, vector):
    """Rotate a 3-tuple by a Quaternion."""
    x, y, z = vector
    qx, qy, qz, qw = quaternion.x, quaternion.y, quaternion.z, quaternion.w

    # t = 2 * (q_vec x v); v' = v + qw * t + q_vec x t
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def make_pose(position, orientation=None):
    """Build a Pose from a 3-tuple position and an optional Quaternion."""
    pose = Pose()
    pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
    pose.orientation = orientation if orientation is not None else Quaternion(w=1.0)
    return pose


def offset_pose(pose, delta):
    """Return a copy of ``pose`` translated by ``delta`` in the planning frame."""
    return make_pose(
        (pose.position.x + delta[0], pose.position.y + delta[1], pose.position.z + delta[2]),
        pose.orientation,
    )


def tcp_to_tip(tcp_pose, tcp_offset):
    """Convert a desired grasp-centre (TCP) pose into a pose for the IK tip link.

    ``tcp_offset`` is the TCP position expressed in the tip link frame, so the
    tip link has to sit at ``tcp - R(tcp) * offset``. The orientation of the TCP
    frame and the tip link frame are identical by construction.
    """
    dx, dy, dz = rotate_vector(tcp_pose.orientation, tcp_offset)
    return make_pose(
        (tcp_pose.position.x - dx, tcp_pose.position.y - dy, tcp_pose.position.z - dz),
        tcp_pose.orientation,
    )
