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

# One lap per scene. The warehouse lap is the original full perimeter; the
# shopfloor (智能制造中心) lap stays in the bench aisle, because west of
# x = -0.8 belongs to the hall's machine row, x = 3.8 is the east wall, and
# the bench slabs occupy x in [2.1, 2.8] around each station's y. The machine
# row is mapped from the aisle - as close as the real robot would ever get.
LAPS = {
    "warehouse": {
        "waypoints": [
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
        ],
        "home": (-7.0, -7.0, 0.67),
    },
    "center": {
        "waypoints": [
            (1.2, -7.6),
            (2.9, -6.2),
            (1.2, -6.0),
            (1.2, -2.0),
            (2.9, -1.8),
            (1.2, -1.6),
            (1.2, 2.0),
            (2.9, 2.4),
            (1.2, 2.6),
            (1.2, 6.2),
            (1.2, 7.6),
            (0.2, 5.4),
            (0.2, -6.5),
        ],
        "home": (0.0, -7.0, 0.67),
    },
}
WAYPOINTS = LAPS["warehouse"]["waypoints"]
HOME = LAPS["warehouse"]["home"]


def main():
    import argparse
    import sys

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default="warehouse", choices=sorted(LAPS))
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    lap = LAPS[args.scene]
    waypoints, home = lap["waypoints"], lap["home"]

    rclpy.init()
    node = Node("mapping_drive")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    base = MobileBase(node, position_error=0.0, yaw_error=0.0)
    base.set_pose(*home)
    node.get_logger().info(f"driving the {args.scene} mapping lap")

    try:
        time.sleep(3.0)  # let slam_toolbox see the robot standing still first
        x, y, _ = home
        for target_x, target_y in waypoints:
            yaw = math.atan2(target_y - y, target_x - x)
            base.drive_to(x, y, yaw, exact=True)  # turn in place first
            base.drive_to(target_x, target_y, yaw, exact=True)
            x, y = target_x, target_y
            time.sleep(1.0)
        base.drive_to(*home, exact=True)
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
