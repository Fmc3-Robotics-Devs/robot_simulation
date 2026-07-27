# Franzi ROS 2 / MoveIt 工作空间

本工作空间用于在 **ROS 2 Jazzy** 中加载和可视化 Franzi 机器人，并提供
MoveIt 2 运动规划配置。

## 内容介绍

- `src/franzi_description`：机器人 URDF、STL 网格模型以及关节名称配置。
- `src/franzi_moveit_config`：MoveIt 2 配置，包括运动学、关节限制、规划器、
  RViz 配置和启动文件。
- `build/`、`install/`、`log/`：由 `colcon build` 生成的构建、安装和日志目录。

当前配置面向固定底座的规划与可视化；默认不启动真实机器人控制器，也不执行
轨迹。

## 前置条件

- Ubuntu 上已安装 ROS 2 Jazzy。
- 已安装 MoveIt 2 及其 Jazzy 依赖。

## 构建

在工作空间根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select franzi_description franzi_moveit_config
```

## ROS 2 Jazzy 环境加载

每个新终端在启动前加载 ROS 2 和本工作空间环境：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

若尚未构建成功，第二条命令会不可用，请先完成上面的构建步骤。

## 启动方式

在工作空间根目录加载环境后，可使用下列 ROS 2 launch 脚本。

### 完整 MoveIt + RViz 演示（推荐）

启动机器人状态发布、静态 TF、MoveIt `move_group` 和 RViz：

```bash
ros2 launch franzi_moveit_config demo.launch.py
```

不启动 RViz：

```bash
ros2 launch franzi_moveit_config demo.launch.py use_rviz:=false
```

### 分别启动各组件

```bash
# 机器人状态发布（robot_state_publisher）
ros2 launch franzi_moveit_config rsp.launch.py

# MoveIt 规划服务（move_group）
ros2 launch franzi_moveit_config move_group.launch.py

# MoveIt RViz 界面
ros2 launch franzi_moveit_config moveit_rviz.launch.py

# 虚拟关节的静态 TF
ros2 launch franzi_moveit_config static_virtual_joint_tfs.launch.py
```

`demo.launch.py` 默认使用 OMPL 规划器，同时配置了 CHOMP 和 Pilz 规划器。
演示默认使用虚拟 `FollowJointTrajectory` 控制器，因此可以在 RViz 中点击
**Execute** 并观察机器人按规划轨迹运动；该控制器只更新可视化关节状态，不会
向任何真实硬件发送命令。

演示启动时会自动发布初始关节状态，使 MoveIt 可以取得有效的当前姿态。接入
`ros2_control` 或仿真器后，它们应负责发布 `/joint_states` 和轨迹 action，此时请
关闭演示驱动：

```bash
ros2 launch franzi_moveit_config demo.launch.py publish_demo_joint_states:=false
```

## 常见问题

若提示 `ModuleNotFoundError: No module named 'catkin_pkg'`，通常是 Conda Python
覆盖了 ROS 使用的系统 Python。请退出 Conda 后清理 CMake 缓存并重新构建：

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-clean-cache \
  --packages-select franzi_description franzi_moveit_config
```

如果终端中预先设置的动态库环境变量影响 ROS/Qt，可使用一个干净的 shell 启动：

```bash
env -u LD_LIBRARY_PATH -u LD_PRELOAD -u LD_AUDIT bash -lc \
  'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch franzi_moveit_config demo.launch.py'
```
