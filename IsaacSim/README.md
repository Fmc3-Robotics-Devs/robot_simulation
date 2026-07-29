# Isaac Sim 运行时说明

本项目唯一运行组合为项目 `.venv` 的 **Python 3.11.14**、**Isaac Sim 5.1.0.0** 与 **Isaac Lab 2.3.2**。使用 Isaac Lab external-project 结构，项目扩展位于 `source/franzi_sim`，不修改 Isaac Sim 或 Isaac Lab 的安装目录。

## 复现环境与本地资产

```bash
uv sync --frozen
export ISAAC_ASSET_ROOT=/home/fmc3/FermiBotNas/SIM_ASSETS/5.1.0/Assets/Isaac/5.1
uv run python scripts/preflight_assets.py --mount
uv run isaacsim isaacsim.exp.compatibility_check
```

预检验证 `Isaac/`、`NVIDIA/` 和 Warehouse 资产，并创建 `vendor/isaac_assets` 相对挂载。该目录只链接本地官方资产，不纳入版本控制；Python 和 USD 源码不得写入机器绝对资产路径。

## 正式场景、物理与证据

[`usd/scenes/warehouse_box_transfer.usda`](../usd/scenes/warehouse_box_transfer.usda) 是当前正式场景，不是 foundation 占位层。它以 NVIDIA Warehouse 作为只读环境，组合了 Unitree 式多工位仓储布局、Wheel Bot、蓝色运输箱、Tag 和放置工位。蓝箱正面绑定 `tag36h11` ID 0，结构检查结果为 [`warehouse_workcell.json`](../evidence/usd/2026-07-29/warehouse_workcell.json)。

蓝箱保留 NVIDIA 的视觉资产，并只添加一个项目控制的 `PhysicsCollision` 代理；箱体是动态刚体。正式场景的重力、台面接触、Tag 跟随与 reset 均已通过 headless smoke，结果见 [`box_gravity_contact_reset.json`](../evidence/physics/2026-07-29/box_gravity_contact_reset.json)。这证明物理场景基础可用，但不等同于双臂抓取/搬运任务已经完成。

```bash
env -u PYTHONPATH -u CMAKE_PREFIX_PATH OMNI_KIT_ACCEPT_EULA=YES \
  uv run --frozen python scripts/verify_usd_stage.py \
  usd/scenes/warehouse_box_transfer.usda \
  --output evidence/usd/$(date +%F)/warehouse_workcell.json

env -u PYTHONPATH -u CMAKE_PREFIX_PATH OMNI_KIT_ACCEPT_EULA=YES \
  uv run --frozen python scripts/capture_workcell_evidence.py \
  --scene usd/scenes/warehouse_box_transfer.usda \
  --output-dir evidence/workcell/$(date +%F)/manual --headless

env -u PYTHONPATH -u CMAKE_PREFIX_PATH OMNI_KIT_ACCEPT_EULA=YES \
  uv run --frozen python scripts/verify_box_physics_smoke.py \
  --scene usd/scenes/warehouse_box_transfer.usda \
  --output evidence/physics/$(date +%F)/box_gravity_contact_reset.json
```

## 相机与 ROS 2 验证

头、胸、左腕、右腕四个 RGB 相机均由真实机器人 link 承载，分辨率为 1280×720，clipping 为 `0.05–100 m`。正式场景的四路 RTX 图和总览位于 [`evidence/workcell/2026-07-29/final/`](../evidence/workcell/2026-07-29/final/)；`head_d435` 已在正式场景画面中解码 Tag 0。当前胸部和双腕画面已经有效，但未在现有任务姿态中覆盖标签，后续抓取姿态阶段再做检测覆盖率验收。

`scripts/verify_ros2_camera_topics.py` 已实现严格同步：每个相机只有一对 `Image` 与 `CameraInfo` 的 `header.stamp.sec` 和 `header.stamp.nanosec` 完全一致时才通过。它尚未针对**最新正式场景**重新采集 live ROS 2、TF 和 Tag 位姿闭环证据；旧 `ros2_topics.json` 仅是历史采集，不得解读为当前同步门已通过。

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 scripts/verify_ros2_camera_topics.py \
  --timeout 45 \
  --output evidence/cameras/$(date +%F)/formal_ros2/ros2_topics.json
```

系统 ROS 2 Jazzy 保持在系统 Python 3.12 环境中；项目 Python 3.11 通过 Isaac Sim ROS 2 Bridge/DDS 通信。不要将 `/opt/ros/jazzy` 的 Python 包或 `rclpy` 安装进 `.venv`。

当前尚未通过的 P3 门是固定底盘的双臂预抓、夹持、抬升、放置、撤离执行器，以及 `≤0.01 rad` 的机械臂跟踪误差。阶段计划与交付说明见 [实施计划](../docs/implementation_plan.md)；汇报材料见 [PPT v0.2](../report/WheelBot仿真项目计划汇报_v0.2.pptx) 和 [PDF v0.2](../report/WheelBot仿真项目计划汇报_v0.2.pdf)。
