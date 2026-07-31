# Franzi LeRobot v3 数据管线

这里保存 Isaac Sim 数据采集约定、物理随机化配置，以及两条生成
LeRobot v3.0 数据集的路径：带 telemetry + subtask 标签的**训练数据采集**
（下节），和从旧录像切视觉样例的遗留工具（文末）。

## 训练数据采集（telemetry + subtask 标签 + 物理参数）

一条命令跑一批：每条 episode 起一次 Isaac（`--physics-episode` 按
`base_seed+index` 确定性采样八项物理量并把工件质量/摩擦/恢复系数写进
stage）+ 一次完整 tending 任务栈，ROS 侧 `collect_episode.py` 按墙钟
同步采四相机 JPEG、54 维 state、25 维 action（下一控制拍的命令），
并逐帧记录 `task/state` 状态机状态。Isaac 用 `--capture dataset`
（所有相机按 640×360 渲染换帧率）：

```bash
COLLECT_FPS=5 IssacSim/lerobot/collect_batch.sh 0 5 45  # 序号 0 起 5 条,DOMAIN 45
# 并行:另开终端错开序号、换 DOMAIN_ID,如 COLLECT_FPS=5 collect_batch.sh 5 5 46
# 采样率约定:先 ros2 topic hz 实测相机吞吐,COLLECT_FPS 取其以下
```

原始产物在 `IssacSim/datasets/episodes/episode_XXXX/`（帧 JPEG +
`telemetry.npz` + `meta.json`），日志在 `episodes/logs/`。然后打包成
多 episode LeRobot v3.0 数据集（h264；逐帧 `subtask_index` 列由状态机
状态经 `subtask_labels.py` 映射，边界即状态转移；同时写
`meta/subtasks.parquet`、`meta/subtask_segments.csv`、
`meta/physics_randomization.json`）：

```bash
conda run -n lerobot-groot python \
  IssacSim/lerobot/build_lerobot_v3_dataset.py \
  --episodes IssacSim/datasets/episodes \
  --output IssacSim/datasets/franzi_machine_tending_physx_v3 \
  --repo-id local/franzi_machine_tending_physx_v3

conda run -n lerobot-groot python \
  IssacSim/lerobot/validate_lerobot_v3.py \
  --root IssacSim/datasets/franzi_machine_tending_physx_v3 \
  --repo-id local/franzi_machine_tending_physx_v3 \
  --require-actions --decode-sample
```

诚实边界：当前仍是 kinematic 镜像阶段——工件质量/摩擦/恢复系数已真实
authored 到 stage 但不改变镜像运动，关节刚度/阻尼/力矩与重力仅记录在
元数据（`physics_applied` / `physics_recorded_only`），等 README 尾部的
物理阶段落地后才生效。失败条（FAULT/超时）默认不进数据集，
`--include-failed` 可收。

## 重要边界

`renders/run_20260729_223604` 只有视频，没有 `/joint_states`、底盘状态、
控制 action、相机时间戳或成功标签。它可以验证四相机和 LeRobot 文件格式，
但不能直接训练 GR00T 的 action head。转换器默认拒绝这种输入，只有显式传
`--visual-only` 才会生成带警告和 provenance 的视觉样例。

训练数据必须在 PhysX 步进时同步记录：

- 四路 RGB：`overhead`、`head_d435`、`left_wrist_d405`、
  `right_wrist_d405`，统一为 640×360；采样率必须不高于 Isaac 实际渲染
  吞吐（`ros2 topic hz` 实测。本机 640×360×4 相机约 5.6-7.4 Hz，故当前
  数据集为 5 FPS；15 FPS 的名义值只在渲染跟得上时使用）；
- `observation.state`：图像时刻测得的 54 维状态；
- `action`：下一个控制周期实际施加给 PhysX 的 25 维命令；
- 英文任务提示词、成功标志和本 episode 的物理随机化实值。

精确定义在 `franzi_groot_schema.json`。54 维 state 和 25 维 action 分别
低于本机 LeRobot GR00T 集成的默认上限 64 和 32。

## PhysX 与并行环境

`physics_collection.json` 使用 PhysX TGS、120 Hz 物理、15 Hz 控制与渲染，
初始并发为 4 个环境。全局基础随机种子是 `12345678`。每条 episode 用
`base_seed + episode_index` 得到可复现种子，并随机化恰好八项物理量：
工件质量、静/动摩擦、恢复系数、关节刚度、关节阻尼、执行器力矩和重力。

查看第 0 条 episode 的实际采样值：

```bash
conda run -n env_isaaclab python \
  IssacSim/lerobot/sample_physics_params.py --episode-index 0
```

大规模采集应采用一个 Isaac Lab 进程中的向量化环境；每个环境拥有独立
scene、robot、四相机、随机种子和 episode buffer。完整 ROS 2 / MoveIt
回归测试才采用多进程，并为每个进程设置不同的 `ROS_DOMAIN_ID`。不要在
同一个 ROS graph 中直接复制当前 `ros2_cell.py`，因为它使用固定的
`/World/Robot` prim 和全局 topic。

## 生成现有录像的视觉格式样例

下列命令只生成视觉 smoke test，并明确标记为 `training_ready: false`：

```bash
conda run -n lerobot-groot python \
  IssacSim/lerobot/convert_run_to_lerobot_v3.py \
  --input-run IssacSim/renders/run_20260729_223604 \
  --output IssacSim/datasets/franzi_machine_tending_visual_v3 \
  --repo-id local/franzi_machine_tending_visual_v3 \
  --visual-only
```

默认四相机是上帝/CCTV、头部、左腕和右腕。旧四宫格的 tile 顺序没有随
录像保存，映射是根据画面推断的；转换后会写入
`meta/provenance.json`。

快速测试可加 `--max-frames 30`。输出目录必须不存在；脚本不会覆盖已有
数据集。

## 带 telemetry 的训练 episode

采集器应提供一个 NPZ：

- `state`: `(N, 54)` float32；
- `action`: `(N, 25)` float32；
- `timestamps_s`: `(N,)` 严格递增时间戳；
- `state_names`、`action_names`: 与 schema 一致的 Unicode 数组。

然后运行：

```bash
conda run -n lerobot-groot python \
  IssacSim/lerobot/convert_run_to_lerobot_v3.py \
  --input-run /path/to/run \
  --output /path/to/fresh_dataset \
  --repo-id local/franzi_machine_tending_physx_v3 \
  --telemetry /path/to/telemetry.npz \
  --action-semantics \
  "PhysX-applied base twist, joint position targets, and gripper width targets"
```

如果 telemetry 与视频起点不同，使用 `--telemetry-offset-s` 显式对齐。
没有时间戳时，telemetry 行数必须与视频帧数完全相等。

## 验证

视觉样例：

```bash
conda run -n lerobot-groot python \
  IssacSim/lerobot/validate_lerobot_v3.py \
  --root IssacSim/datasets/franzi_machine_tending_visual_v3 \
  --repo-id local/franzi_machine_tending_visual_v3 \
  --decode-sample
```

训练数据额外加 `--require-actions`。验证会检查 v3.0、单 episode、四相机
等分辨率、state/action 是否存在，并用 PyAV 真解码一帧。
