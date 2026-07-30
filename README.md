# Wheel Bot Isaac Sim

Wheel Bot 的 Isaac Sim 工程基线：项目 `.venv` 使用 **Python 3.11.14**，锁定 **Isaac Sim 5.1.0.0** 与 **Isaac Lab 2.3.2**。依赖按锁文件复现，统一使用 `uv sync --frozen` 和 `uv run`。

## 当前可验证状态（2026-07-30）

正式入口为 [`usd/scenes/warehouse_box_transfer.usda`](usd/scenes/warehouse_box_transfer.usda)：一个 Unitree 风格的多工位仓储工位，基于 NVIDIA 官方 Warehouse，包含 Wheel Bot、两张作业台、三处背景工位、蓝色运输箱和放置区。蓝箱正面贴有项目自有的 `tag36h11` **ID 0** AprilTag；正式场景由 [`warehouse_workcell.json`](evidence/usd/2026-07-30/warehouse_workcell.json) 验证。

四路相机（头部、胸部、左腕、右腕）均为 1280×720、`0.05–100 m` clipping，已从该正式场景取得有效 RTX 图像；复核板见 [`workcell_review_board.png`](evidence/workcell/2026-07-30/final/workcell_review_board.png)。D405 的 USD 光轴已从错误的 housing `+X` 修正为 STL 镜头面的 housing `-Z`，并把 USD Camera frame 与 ROS optical frame 分离；左右腕 D405 均在限位内的相机复核种子姿态中直接解码蓝箱 Tag 0。该姿态只用于传感器取证，不冒充运动规划得到的预抓轨迹。

机器人导入资产当前有 **37 个有效碰撞体，全部为 `convexHull`**，不是 `convexDecomposition`。固定底盘场景按最低轮面实测值把机器人根节点抬高到 `z=0.0873 m`：三轮最低碰撞点距仓库 `z=0` 地面约 `0.034 mm`，底盘本体净空约 `34.7 mm`。验证脚本会遍历 instance proxy，检查碰撞数量、近似类型、三轮同高和地面间隙，避免只看可见网格误判。

蓝箱采用 NVIDIA 视觉资产叠加项目**单一** `PhysicsCollision` 代理；刚体、质量和摩擦参数由项目物理层提供。正式场景的 gravity、接触落稳、Tag 随箱、reset 冒烟测试已通过，见 [`box_gravity_contact_reset.json`](evidence/physics/2026-07-30/box_gravity_contact_reset.json)。

尚未完成的范围是固定底盘双臂搬箱执行器，以及 `≤0.01 rad` 的机械臂跟踪门。ROS 2 的 Image/CameraInfo **精确同时间戳**验证器已经实现；但最新正式场景的 live ROS 2 与 TF/Tag 位姿闭环尚未重新采集，历史 ROS 证据不能当作该门已通过。

实施范围、阶段门和接口约定见 [实施计划](docs/implementation_plan.md)；汇报材料为 [PPT v0.2](report/WheelBot仿真项目计划汇报_v0.2.pptx) 与 [PDF v0.2](report/WheelBot仿真项目计划汇报_v0.2.pdf)。运行时、素材挂载与 ROS 边界见 [Isaac Sim 说明](IsaacSim/README.md)。

## 安装与资产

```bash
uv sync --frozen
export ISAAC_ASSET_ROOT=/home/fmc3/FermiBotNas/SIM_ASSETS/5.1.0/Assets/Isaac/5.1
uv run python scripts/preflight_assets.py --mount
```

本机官方资产根必须包含 `Isaac/` 和 `NVIDIA/`。预检会创建仓库内的 `vendor/isaac_assets` 相对挂载；官方资产不提交进 Git。

可选的 Hugging Face Isaac Sim/Isaac Lab 数据只用于资产、轨迹结构和多相机记录参考，不是正式场景依赖。已核对许可证并固定版本的候选见 [技术参考登记](docs/references.md)，缓存、审核与离线规则见 [资产约定](docs/assets.md)；大文件与审核快照放入项目自定义的 `HF_ASSET_ROOT`（本机建议 `/home/fmc3/FermiBotNas/SIM_ASSETS/huggingface`），仓库只通过忽略提交的 `vendor/huggingface` 相对挂载访问，正式场景不在启动时联网拉取。

确认项目解释器与锁定的运行时版本：

```bash
uv run python --version
uv run python -c "from importlib.metadata import version; print('isaacsim', version('isaacsim')); print('isaaclab', version('isaaclab'))"
uv run isaacsim isaacsim.exp.compatibility_check
```

## 验证

不依赖 GPU 的项目测试：

```bash
uv lock --check
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest
uv run python scripts/verify_camera_configuration.py
```

让 Isaac Sim 打开并验证正式仓储工位：

```bash
env -u PYTHONPATH -u CMAKE_PREFIX_PATH OMNI_KIT_ACCEPT_EULA=YES \
  uv run --frozen python scripts/verify_usd_stage.py \
  --output evidence/usd/$(date +%F)/warehouse_workcell.json
```

重新采集正式场景的总览、AprilTag 近景和四路 RTX 画面：

```bash
env -u PYTHONPATH -u CMAKE_PREFIX_PATH OMNI_KIT_ACCEPT_EULA=YES \
  uv run --frozen python scripts/capture_workcell_evidence.py \
  --scene usd/scenes/warehouse_box_transfer.usda \
  --output-dir evidence/workcell/$(date +%F)/manual --headless
```

运行蓝箱重力、接触、Tag 跟随和 reset 冒烟测试：

```bash
env -u PYTHONPATH -u CMAKE_PREFIX_PATH OMNI_KIT_ACCEPT_EULA=YES \
  uv run --frozen python scripts/verify_box_physics_smoke.py \
  --scene usd/scenes/warehouse_box_transfer.usda \
  --output evidence/physics/$(date +%F)/box_gravity_contact_reset.json
```

在项目 Isaac Sim 环境启动正式场景 ROS 2 Bridge 后，用**系统 ROS 2 Jazzy**终端执行同步门。它只接受 `Image` 与 `CameraInfo` 的 `sec`、`nanosec` 完全相同的一对消息：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 scripts/verify_ros2_camera_topics.py \
  --timeout 45 \
  --output evidence/cameras/$(date +%F)/formal_ros2/ros2_topics.json
```

不要把系统 Jazzy 的 Python 3.12 包注入项目 `.venv`；两端经 DDS/Isaac Sim ROS 2 Bridge 通信。`OMNI_KIT_ACCEPT_EULA=YES` 只表示当前用户已经接受 NVIDIA EULA，不应在未阅读条款时使用。
