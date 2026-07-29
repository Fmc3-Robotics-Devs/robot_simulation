#!/usr/bin/env python3
"""Drive the base around the cell so slam_toolbox can build the map.

Simulation's stand-in for the joystick lap a person drives on the real cell:
a rectangle through the aisle in front of the benches, facing the direction
of travel, ending back at the standby point. The base publishes /odom and the
odom TF while it moves; Isaac mirrors it and its lidar publishes /scan, which
is all slam_toolbox needs.

    ros2 run franzi_skills mapping_drive

Keeps publishing after the lap so the map can be saved; stop with Ctrl-C.
"""

import math
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from franzi_pick_place.base import MobileBase

# The full-perimeter lap, driven for real rather than trusting long-range
# scan returns: out of the wall corner, along the south wall, up the bench
# aisle (x = 1.2 clears the legs at x >= 2.0), along the north wall, down the
# west side (x = -6 clears the racks), back to the corner. Every area the
# robot will ever drive gets close-range, multi-angle coverage.
WAYPOINTS = [
    (-2.0, -7.0),
    (0.8, -7.0),
    (1.2, -4.8),
    (1.2, 0.0),
    (1.2, 4.8),
    (0.8, 6.8),
    (-2.0, 6.8),
    (-6.0, 6.8),
    (-6.0, 0.0),
    (-6.0, -6.2),
    (-7.0, -7.0),
]
HOME = (-7.0, -7.0, 0.67)


def main():
    rclpy.init()
    node = Node("mapping_drive")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    base = MobileBase(node, position_error=0.0, yaw_error=0.0)
    base.set_pose(*HOME)
    node.get_logger().info("driving the mapping lap")

    try:
        time.sleep(3.0)  # let slam_toolbox see the robot standing still first
        x, y, _ = HOME
        for target_x, target_y in WAYPOINTS:
            yaw = math.atan2(target_y - y, target_x - x)
            base.drive_to(x, y, yaw, exact=True)  # turn in place first
            base.drive_to(target_x, target_y, yaw, exact=True)
            x, y = target_x, target_y
            time.sleep(1.0)
        base.drive_to(*HOME, exact=True)
        node.get_logger().info("lap done - save the map, then Ctrl-C")
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
