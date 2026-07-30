# Wheel Bot 机器人 USD

`wheel_bot.usd` 及 `configuration/` 是完整、可随项目交付的 Isaac Sim 5.1 机器人资产。源模型从本仓库 `origin/franzi` 的以下版本选择性复用：

```text
commit 7457a17ead2a7ab199c84a281f0ab90824abc7ef
Rviz/src/franzi_description/urdf/wheel_robot_4.0.urdf
Rviz/src/franzi_description/meshes/*.STL
```

`origin/franzi` 版本作为来源记录保留；项目交付的 URDF 仅把左右 `wrist_d405_joint` 的 RPY 从旧 `Rx(180°)` 修正为 identity，使 D405 实体镜头面 `-Z` 与 wrist-roll/夹爪 `-Z` 同轴。由于 USD prim 名不能以数字开头，导入脚本还会在临时副本中把 `2Dlidar_Link` / `2Dlidar_joint` 改为 `lidar_2d_Link` / `lidar_2d_joint`，避免 Isaac Sim 5.1 URDF importer 的 `Used null prim` 错误；这一临时重命名不回写项目 URDF。

在已经完成 `uv sync --frozen` 的项目环境中重新生成：

```bash
uv run python scripts/import_robot_urdf.py --headless
```

`config.yaml` 记录源 commit、项目方向修正、Isaac Sim 5.1 原生 URDF importer 2.4.30、关节驱动和碰撞策略；其中只使用项目相对路径，不保留生成机器的绝对路径。`.asset_hash` 是交付 URDF 与该配置的 SHA-256 指纹。当前固定底盘基线显式使用：

```yaml
fix_base: true
collision_from_visuals: false
collider_type: convex_hull
self_collision: false
```

原 URDF 为 37 个 link 提供独立 collision mesh，正式组合场景实测为 37 个 enabled `CollisionAPI`，全部 `physics:approximation=convexHull`。不要把整机直接改为 `convex_decomposition`：移动底盘阶段优先为三轮建立 cylinder/单凸包代理，为底座和手臂建立少量简化凸体，只在夹指等接触敏感部位局部提高精度。依据见 [NVIDIA Physics Fundamentals](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/physics/simulation_fundamentals.html)。
