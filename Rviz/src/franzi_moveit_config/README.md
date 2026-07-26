# Franzi MoveIt configuration

This package provides fixed-base, planning-only MoveIt 2 configuration for the
Franzi robot on ROS 2 Jazzy.

Configured planning groups:

- `body`, `head`, and composite `body_head`
- `left_arm`, `right_arm`, and composite `both_arms`
- `left_gripper` and `right_gripper`

The demo deliberately disables trajectory execution because no `ros2_control`
or physical-robot controller is launched. `moveit_controllers.yaml` only
declares the expected action names for later integration. The joint-limit
overrides are conservative planning defaults and must be checked against the
controller before any physical execution.

Build:

```bash
source /opt/ros/jazzy/setup.bash && cd /home/hanchong/FMC3/robot_simulation/Rviz && colcon build --symlink-install --packages-select franzi_description franzi_moveit_config
```

Launch the planning-only RViz demo:

```bash
env -u LD_LIBRARY_PATH -u LD_PRELOAD -u LD_AUDIT bash -lc 'source /opt/ros/jazzy/setup.bash && source /home/hanchong/FMC3/robot_simulation/Rviz/install/setup.bash && QT_QPA_PLATFORM=xcb ros2 launch franzi_moveit_config demo.launch.py'
```
