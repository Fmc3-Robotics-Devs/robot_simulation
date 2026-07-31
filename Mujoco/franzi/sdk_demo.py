"""Exercise the vendor-compatible SDK surface end to end.

Run:  python3 sdk_demo.py    (needs the mujoco pip package)
"""

import math

from franzi_sim_sdk import HEAD, WAIST, FranziSimClient

robot = FranziSimClient()
assert robot.IsReady() and robot.AxisCount() == 7

# MoveJ through two waypoints on the left arm, vendor-style status polling.
assert robot.MoveJ(
    [[-0.4, 0.3, 0.0, -0.6, 0.0, 0.0, 0.0],
     [-0.8, 0.3, 0.0, -1.2, 0.0, 0.3, 0.0]],
    side="left",
) == 0
assert robot.GetMotionStatus("left") == 1
while robot.GetMotionStatus("left"):
    robot.step()
robot.spin(0.5)
reached = robot.GetCurJointPos("left")
for actual, target in zip(reached, [-0.8, 0.3, 0.0, -1.2, 0.0, 0.3, 0.0]):
    assert abs(actual - target) < 0.05, (reached,)
print("MoveJ left arm: reached", [round(v, 3) for v in reached])

# Instant dual-arm command (14-dim) plus head and waist unit targets.
assert robot.CommandJointPosition([0.0] * 14) == 0
assert robot.CommandUnitPosition(HEAD, [0.5, 0.2]) == 0
assert robot.CommandUnitPosition(WAIST, [0.0, 0.0, 0.0, 0.4]) == 0
robot.spin(1.5)
assert abs(robot.GetCurJointPos(HEAD)[0] - 0.5) < 0.05
assert abs(robot.GetCurJointPos(WAIST)[3] - 0.4) < 0.05
print("head/waist units track")

# Gripper: close, read back, open.
robot.GripperClose("right")
robot.spin(0.8)
assert robot.GripperReadPosition("right") > 0.9
robot.GripperOpen("right")
robot.spin(0.8)
assert robot.GripperReadPosition("right") < 0.1
print("gripper close/open round-trip")

# Chassis: forward, then sideways (holonomic), then yaw in place.
x0, y0, _ = robot.GetBasePose()
robot.CommandBaseVelocity(0.3, 0.0, 0.0)
robot.spin(2.5)
robot.CommandBaseVelocity(0.0, 0.25, 0.0)
robot.spin(1.0)  # let the steering modules swing 90 degrees first
x1, y1, _ = robot.GetBasePose()
robot.spin(2.0)
robot.CommandBaseVelocity(0.0, 0.0, 0.5)
robot.spin(2.5)
x2, y2, yaw = robot.GetBasePose()
assert x1 - x0 > 0.5, f"forward leg moved only {x1 - x0:.2f} m"
assert y2 - y1 > 0.3, f"sideways leg moved only {y2 - y1:.2f} m"
assert abs(yaw) > 0.5, f"yaw leg turned only {math.degrees(yaw):.0f} deg"
print(f"base: forward {x1 - x0:.2f} m, sideways {y2 - y1:.2f} m, "
      f"yaw {math.degrees(yaw):.0f} deg")

# SoftStop rejects commands and freezes the queue.
robot.MoveJ([[0.5, 0, 0, 0, 0, 0, 0]], side="left")
robot.SoftStop(True)
assert not robot.IsReady()
assert robot.MoveJ([[0.5, 0, 0, 0, 0, 0, 0]], side="left") != 0
assert robot.GetMotionStatus("left") == 0
robot.SoftStop(False)
assert robot.Home() == 0
print("softstop + home ok")
print("sdk demo passed")
