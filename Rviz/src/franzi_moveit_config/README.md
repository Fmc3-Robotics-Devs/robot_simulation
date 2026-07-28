# Franzi MoveIt configuration

This package provides fixed-base, planning-only MoveIt 2 configuration for the
Franzi robot on ROS 2 Jazzy.

Configured planning groups:

- `body`, `head`, and composite `body_head`
- `left_arm`, `right_arm`, and composite `both_arms`
- `left_gripper` and `right_gripper`

The gripper `open` / `closed` named states follow the finger geometry: the two
finger joints sit 95 mm apart and travel towards each other, so joint zero is
fully open and the travel limits (0.0475 / -0.0475) are fully closed.

The demo includes virtual `FollowJointTrajectory` controllers, so planned
motion is animated in RViz without sending commands to hardware. The virtual
controllers also publish the current joint states required by MoveIt. They are
for visualization only; the joint-limit overrides are conservative planning
defaults and must be checked against a real controller before physical use.

Build, from the `Rviz/` workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select franzi_description franzi_moveit_config
```

Launch the RViz demo with virtual trajectory execution:

```bash
env -u LD_LIBRARY_PATH -u LD_PRELOAD -u LD_AUDIT bash -lc \
  'source /opt/ros/jazzy/setup.bash && source install/setup.bash \
   && QT_QPA_PLATFORM=xcb ros2 launch franzi_moveit_config demo.launch.py'
```

When integrating a real controller or simulator, do not launch the demo
driver. It would otherwise publish duplicate joint states and action servers:

```bash
ros2 launch franzi_moveit_config demo.launch.py publish_demo_joint_states:=false
```
