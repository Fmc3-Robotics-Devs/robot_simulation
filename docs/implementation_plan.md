# Wheel Bot 仿真项目实施计划（搬箱优先）

## 1. 目标与已确认决策

本项目以 Isaac Sim 5.1.0.0 与 Isaac Lab 2.3.2 为唯一首版运行组合，采用 `uv` 管理 Python 3.11 项目 `.venv`。三个目标场景依次为**移动底盘双臂搬箱**、**手机扫码装箱**和**机床上下料**：先完成搬箱闭环，再实现手机场景；机床上下料本阶段只交付详细技术计划。

已确认的交付原则如下：

- 交付代码、项目自有的完整 USD 组合层、README 和本实施计划；代码保持简洁、可扩展，并为每个功能提供必要的责任说明和“为什么”注释。
- NVIDIA 官方素材不复制进 Git：通过 `vendor/isaac_assets` 的相对挂载引用。项目自己的机器人、货箱、泡沫箱、手机、传感器和场景 USD 全部提交到仓库，保证在按 README 挂载官方素材后可以直接打开顶层场景。
- 场景优先使用本地 Isaac 5.1 官方仓库/周转箱资产；官方文档与社区仓库只作为有版本记录的参考，不整仓复制第三方工程。
- 资料优先级为 NVIDIA 5.1 官方文档与本地资产、NVIDIA Hugging Face 固定版本数据、许可证清楚的社区 Hugging Face/GitHub 仓库。HF 用于资产候选、轨迹/数据结构和相机/随机化参考，不直接替代本项目 USD、关节映射或实测验收。
- 手机和泡沫箱的最终尺寸、碰撞几何以提供的 STL 为准；`report/手机放置槽.STL` 就是泡沫箱，当前模型为 3 列 × 6 行、共 18 槽；手机模型来自 `report/i17_AIR_DUMMY_stls/`。
- 机器人须提供头部、胸部、左腕、右腕四路相机，并能取得实际渲染画面；仅在 USD/URDF 中存在 prim 或 frame 不算通过。
- 项目采用 **subagent 驱动实施**：环境配置、版本核对、资产预检、重复性测试等边界清楚的任务优先交给并行 subagent；主任务负责接口决策、结果集成和最终验收。

本计划是工程的执行基线。实现中发现会改变上述决策、验收范围或对外接口的情况，先在本文件的“变更记录”登记，再与负责人确认。

## 2. 可交付目录与责任边界

```text
robot_simulation/
├── README.md                         # 从零开始的安装、资产挂载、运行、验收说明
├── pyproject.toml / uv.lock           # Python 3.11、Isaac Sim/Lab 的唯一依赖基线
├── docs/
│   ├── implementation_plan.md        # 本计划与执行状态
│   ├── assets.md                     # 来源、尺寸、导入和许可证约定
│   ├── development.md                # 代码与 USD 开发约定
│   └── references.md                 # 官方/社区参考的版本与采用范围
├── source/franzi_sim/
│   └── franzi_sim/
│       ├── scenarios/                # 任务状态机、配置、运行入口
│       ├── cameras.py                # 四相机唯一配置源
│       ├── camera_runtime.py         # 相机 prim / render product 运行时适配
│       └── ros2_camera_bridge.py     # RGB 与 CameraInfo ROS 2 发布图
├── Rviz/src/franzi_description/      # 从 franzi 选择性移植的 URDF、mesh 与配置
├── usd/
│   ├── assets/robots/wheel_bot/      # 机器人、相机层和完整组合入口
│   ├── assets/                       # 后续项目自有货箱、泡沫箱、手机等 USD
│   └── scenes/                       # 可直接打开的顶层场景 USD
├── scripts/                          # 资产预检、导入、启动、截图和验收脚本
├── evidence/                         # 验收截图与运行记录（按任务/日期归档）
├── tests/                            # 无 Isaac 单测与可选 Isaac 集成测试
└── vendor/isaac_assets               # 本地符号链接，忽略提交
```

顶层入口保持为 `usd/scenes/warehouse_box_transfer.usda`。该层引用官方仓库环境并组合项目自有机器人、货箱和传感器层；所有路径必须相对项目根目录，不能写死个人机器的绝对资产路径。

截至 2026-07-30，该入口已形成正式的 Unitree 式多工位 Warehouse 工位：Wheel Bot 位于主作业台，蓝色运输箱位于其前方，交付台与三处背景工位共同提供正常仓储语义。蓝箱正面附着项目自有 `tag36h11` **ID 0**，用于随箱位姿；主 PickTable 近侧右、左两个对称空闲角另组合朝 `+Z` 的 80 mm **ID 1/2**，作为静态工位定位基准。三者必须在检测、TF 和验证输出中按 ID/角色区分。结构、碰撞与四相机坐标基线为 `evidence/usd/2026-07-30/warehouse_workcell.json`，RTX 总览、Tag 0/1/2 独立近景和四相机截图为 `evidence/workcell/2026-07-30/final/`；证据 manifest 同时绑定正式场景、完整机器人配置层和 D405 修正版 URDF。蓝箱的重力、接触、Tag 跟随和 reset smoke 见 `evidence/physics/2026-07-30/box_gravity_contact_reset.json`。这仍不等同于搬箱闭环完成：固定底盘双臂执行器和 `≤0.01 rad` 跟踪门仍在 P3。

## 3. 目标场景与任务接口

### 3.1 首版任务闭环

```text
识别货箱 → 底盘靠近 → 底盘对位 → 双臂抓取 → 抬升
      → 搬运 → 指定位置放置 → 双臂撤离 → 完成
```

现有 `BoxTransferStateMachine` 是任务编排的稳定边界。执行器只上报“本阶段完成”或“本阶段失败”，不得在底盘、MoveIt、ROS 2 或 Isaac 回调中私自跨阶段推进。首版使用下列约束：

| 接口/对象 | 首版职责 | 输入 | 输出 |
|---|---|---|---|
| `RunScenarioRequest` | 启动一次可复现实验 | 场景名、随机种子、可选配置 | 运行句柄/结果 |
| `BoxTransferStateMachine` | 确定性推进与重试 | 执行器确认或失败原因 | 当前阶段、终止状态 |
| `PerceptionExecutor` | 提供货箱/标记检测结果 | 头、胸、腕部相机帧 | 目标位姿与置信信息 |
| `MobileBaseExecutor` | 驱动底盘到预抓、搬运、放置位 | 目标平面位姿 | 到位确认 |
| `DualArmExecutor` | 规划/执行预抓、抓取、抬升、放置、撤离 | 箱体位姿、抓取参数 | 阶段确认 |
| `CameraManager` | 配置、启动、读取和保存四路画面 | 相机名称、分辨率、帧请求 | RGB（必要时深度）帧 |

接口实现可先用固定底盘完成双臂抓取、抬升和放置的内部门槛，随后接入 `MobileBaseExecutor` 达到完整移动搬箱闭环。固定底盘演示不是最终交付替代项。

### 3.2 手机扫码装箱（搬箱完成后实施）

后续导入 `i17_AIR_DUMMY_stls` 手机模型，并以 `report/手机放置槽.STL` 生成的 18 槽泡沫箱 USD 为容器对象。槽位行列数、遍历顺序和安全偏置全部配置化，不写死在动作代码中。任务状态机为：

```text
取手机 → 调整扫码姿态 → 触发扫码
  ├─ PASS → 插入当前槽 → 释放 → 退出 → 持久化槽号 +1
  └─ FAIL/TIMEOUT → 原姿态重试一次
        ├─ PASS → 正常装箱
        └─ 再次失败 → 放入 NG 区，槽号不增加
```

只有“插入、释放、退出”全部确认后才能增加槽号；重启后从持久化状态继续，不允许漏槽或重复。当前 3×6 模型用于首轮导入、碰撞和整箱循环验证；若后续提供其他槽数的正式 STL，以模型和配置为准。

### 3.3 机床上下料（本阶段只交付计划）

计划覆盖“原料区 → 导航对接 → 门/夹具联锁 → 上料 → 加工等待 → 下料 → 成品区”的完整状态机，并给出 TF、Nav2、AprilTag、MoveIt、PLC/IO 和安全监督三层架构。交付物还需列清机床模型、工装尺寸、PLC 点表、节拍和验收方法；本阶段不实现机床仿真代码或真实 PLC 驱动。

## 4. USD、URDF 与四相机方案

### 4.1 资产与层级

1. 预检脚本验证 `ISAAC_ASSET_ROOT=/home/fmc3/FermiBotNas/SIM_ASSETS/5.1.0/Assets/Isaac/5.1`，并创建 `vendor/isaac_assets` 相对挂载。
2. 以官方 Warehouse 和可用官方周转箱为环境/基础资产；若官方箱体尺寸与提供 STL 不一致，以 STL 导入后的项目自有货箱 USD 为碰撞和可抓取对象的依据。
3. 导入泡沫箱和手机时统一执行毫米到米的换算，分别生成可视层、简化碰撞层、物理属性层与组合入口；原始 STL 保持不修改。
4. 机器人 URDF 负责 link、joint、碰撞和四个相机实体外壳 link/fixed joint；机器人 USD 在这些真实 link 下分别补充 USD `*_camera_mount/Camera` 与 ROS `*_optical_frame` sibling、物理属性和场景组合。ROS TF optical-frame 链在 P2 由显式静态变换补齐并验证；三者的 link/frame 名称必须一一映射，并由测试检查。

当前机器人与四相机组合入口为 `usd/assets/robots/wheel_bot/wheel_bot_with_cameras.usda`。实际层级为：

```text
/World/WheelBot
├── head_d435_Link
│   ├── head_d435_camera_mount/camera
│   └── head_d435_optical_frame
├── body_d435_Link
│   ├── chest_d435_camera_mount/camera
│   └── chest_d435_optical_frame
├── left_wrist_d405_Link
│   ├── left_wrist_d405_camera_mount/camera
│   └── left_wrist_d405_optical_frame
└── right_wrist_d405_Link
    ├── right_wrist_d405_camera_mount/camera
    └── right_wrist_d405_optical_frame
```

四路均为 1280×720 RGB。头/胸 D435 的 USD Camera mount 为 `(180°,0°,-90°)`，对应 ROS optical 为 `(0°,0°,-90°)`；D405 STL 镜头面法向是 housing `-Z`，项目修正版 URDF 将两条 D405 fixed joint 设为 identity，使 housing `-Z` 与 wrist-roll/夹爪 `-Z` 同轴。双腕 USD Camera mount 为 `(0°,0°,90°)`，对应 ROS optical 为 `(180°,0°,90°)`；其中 `Rz=90°` 只校正画面 roll，不改变向下光轴。当前单一 RGB Camera 暂建模为两镜头之间、位于 STL 镜头面外 `0.5 mm` 的虚拟双目中点；正式机械图或内外参到位后再标定完整 XYZ 和内参，不把该中点误报为真实 RGB 光心。转换遵循 NVIDIA 官方的 USD `+Y up/-Z forward` 与 ROS optical `+Y down/+Z forward` 约定。ROS 2 Bridge 为每路创建 `IsaacCreateRenderProduct`、`ROS2CameraHelper` 和 `ROS2CameraInfoHelper`，输出：

| 相机 | RGB topic | CameraInfo topic |
|---|---|---|
| 头部 D435 | `/franzi/camera/head_d435/color/image_raw` | `/franzi/camera/head_d435/color/camera_info` |
| 胸部 D435 | `/franzi/camera/chest_d435/color/image_raw` | `/franzi/camera/chest_d435/color/camera_info` |
| 左腕 D405 | `/franzi/camera/left_wrist_d405/color/image_raw` | `/franzi/camera/left_wrist_d405/color/camera_info` |
| 右腕 D405 | `/franzi/camera/right_wrist_d405/color/image_raw` | `/franzi/camera/right_wrist_d405/color/camera_info` |

URDF 已提供四个相机外壳 link 和 fixed joint；USD 在对应真实 link 下添加独立渲染 frame、ROS optical frame 与 Camera prim，不创建脱离刚体链的静态“假相机”。四路均明确设置 `clippingRange=(0.05,100) m`，并在正式 Warehouse 工位取得有效 RTX 图像。双腕以全零自然下垂姿态验收：`camera_forward·housing_-Z ≥ 0.999`、`camera_forward·wrist_roll_-Z ≥ 0.999`、`camera_forward·world_down ≥ 0.999`，画面 roll 符合左右外侧机身布局，且中心光轴向下 0.3 m 内没有近端腕/夹爪的首次碰撞。中立姿态不要求 D405 解码 Tag；箱上 Tag 0、桌角 Tag 1/2 的可见性在独立近景或实际任务姿态分别验收。ROS 2 的 Image/CameraInfo 精确同时间戳验证器已经实现，但最新正式场景的 live ROS 2、TF 和 Tag 位姿闭环尚未重新采集；历史 `ros2_topics.json` 不能声称满足当前同步门。

### 4.2 碰撞与落地基线

机器人 URDF 的 37 个显式 collision mesh 由 Isaac Sim 5.1 自带的官方 URDF importer 2.4.30 以 `convex_hull` 导入；项目没有调用与当前 wheel 不兼容的 Isaac Lab 2.4.31 importer 扩展。`collision_from_visuals=false`、`self_collision=false`、`fix_base=true` 均在导入脚本和 `config.yaml` 中显式固定，不依赖版本默认值。正式 stage 必须用 `Usd.TraverseInstanceProxies()` 审计，否则会漏掉引用资产内的碰撞体。

固定底盘基线不通过求解器“挤出”地面。零位姿最低轮面相对机器人根节点为 `-0.0872656 m`，正式场景将 root 放在 `z=0.0873 m`；当前三轮最低点为地面上约 `0.034 mm`，三轮高度差小于 `0.01 mm`，`base_link` 净空约 `34.7 mm`。自动验证门为：

1. enabled robot collider 恰为 37 个，近似统计只能是 `{"convexHull":37}`；
2. 左前、右前、后轮各一个有效碰撞体，三轮最低点差不超过 `0.1 mm`；
3. 最低机器人碰撞点距 `z=0` 地面绝对值不超过 `0.5 mm`；
4. `base_link` 最低碰撞点必须高于地面。

全局切换 `convex_decomposition` 不是当前方案。P4 移动底盘优先把轮胎改为 cylinder/单凸包，把底座和手臂改为少量手工简化凸体；夹指等接触敏感处才局部采用多凸块或 convex decomposition。动态 articulation 不使用普通 triangle mesh 作为整机默认碰撞。

蓝箱使用一个项目自有的 metre-scale Cube 碰撞代理，不叠加官方视觉资产内部碰撞。箱体中心位于 PickTable `x=1.20 m`，近侧留有约 `28 mm` 完整支撑余量；480 步重力/接触 smoke 的最终水平漂移约 `1.23e-7 m`。旧位置 `x=1.05 m` 曾悬出台面约 `124 mm`，已禁止回退。

### 4.3 相机验收要求

每次生成/修改机器人 USD 或 URDF 后，必须同时完成以下检查：

1. **结构检查**：脚本读取 URDF 与 USD，确认四个相机 frame/prim 全部存在、父 link 有效、名称映射一致，且没有固定绝对路径。
2. **运行检查**：在 Isaac Sim 打开顶层场景、完成物理加载后，通过相机 render product 获取四张 RGB 图。图像非空、尺寸符合配置、同一仿真时刻的时间戳可追溯。
3. **GUI/截图检查**：在 Isaac Sim GUI 逐路打开 viewport，或用同一 RTX 渲染链路批量保存四路截图；人工检查画面方向、遮挡、目标可见性和左右腕归属。修改安装位或旋转后必须重新截图，不能只依赖数值检查。
4. **证据归档**：保存为 `evidence/cameras/YYYY-MM-DD/<run_id>/head_d435.png`、`chest_d435.png`、`left_wrist_d405.png`、`right_wrist_d405.png`，另存 `four_camera_contact_sheet.png` 和 `manifest.json`（场景 USD、提交版本、分辨率、仿真时间、相机路径）。汇报 PPT 仅复用四宫格结果图，不替代原始证据。

截至 2026-07-30，修正后的正式 Warehouse 场景四路 RTX 画面已归档到 `evidence/workcell/2026-07-30/final/` 并完成人工检查；自然下垂时左右 D405 均朝地，且分别把机器人保留在画面外侧。Tag 0/1/2 由独立近景解码，不把中立腕相机画面误报为标签检测结果。历史 ROS 2 采集不再作为通过依据；正式场景仍需用精确时间戳验证器重新采集 4 路 Image/CameraInfo，并完成 TF 与 Tag 位姿闭环。

任何一路只有黑屏、错误相机视角、无 render product、未挂接真实 link，或无法留存截图，均不能通过相机验收。

### 4.4 Hugging Face 数据采用流程

Hugging Face 候选及固定 commit 统一登记在 `docs/references.md`。当前优先评估 NVIDIA G1 locomanipulation 的“抓取—导航—放置”阶段与成功定义、SimReady Warehouse 的 OpenUSD 资产，以及 Isaac Lab Mimic/LeRobot 的示范与多相机数据结构。手机阶段可参考 Unitree 的 Object Placement 与 Camera Packaging 数据，但手机、泡沫箱、槽位和 Wheel Bot 动作仍以本项目模型与实测为准。

```text
数据卡/许可证审查
    → 固定 Hub commit
    → 只下载 README、元数据或最小样本
    → 隔离检查格式、单位、坐标系、关节和 Isaac 版本
    → 转换为 Wheel Bot 项目接口
    → 单测 + Isaac GUI/物理/截图复验
    → 决定采用、仅参考或拒绝
```

HF 数据不得成为正式 USD 的隐式在线依赖。大文件与审核快照存放在 `HF_ASSET_ROOT`，仓库内 `vendor/huggingface/` 只提供忽略提交的相对挂载；正式运行设置离线模式。需要交付的派生 USD、配置或小型测试样本必须有许可证允许、来源说明、固定 revision 和文件哈希。其他机器人动作不能直接发送给 Wheel Bot，必须经过语义阶段提取、关节重定向、限位检查和仿真回归。

| 阶段 | 优先参考 | 只迁移的内容 |
|---|---|---|
| P2/P3 固定底盘搬箱 | G1 Locomanipulation、OpenArm Pick、Franka Mimic | 任务阶段、示范/相机 schema、成功条件 |
| P4 移动搬箱 | G1 Locomanipulation、Synthetic Warehouse Operations | 导航与操作段拼接、随机种子、多视角与场景随机化 |
| P5 手机扫码装箱 | Unitree Object Placement、Camera Packaging | 双臂装盒阶段、头/腕相机组织、LeRobot 元数据 |

## 5. 分阶段实施与质量门

### 5.1 正式场景的可复制验收命令

以下命令以当前唯一正式入口 `usd/scenes/warehouse_box_transfer.usda` 为准；每次修改场景、相机、Tag 或物理层后至少重跑对应项并归档新证据。

```bash
# 结构、四相机 clipping 和箱体/Tag/物理属性
env -u PYTHONPATH -u CMAKE_PREFIX_PATH OMNI_KIT_ACCEPT_EULA=YES \
  uv run --frozen python scripts/verify_usd_stage.py \
  usd/scenes/warehouse_box_transfer.usda \
  --output evidence/usd/$(date +%F)/warehouse_workcell.json

# 正式多工位场景总览、蓝箱 Tag 近景、四路 RTX 画面
env -u PYTHONPATH -u CMAKE_PREFIX_PATH OMNI_KIT_ACCEPT_EULA=YES \
  uv run --frozen python scripts/capture_workcell_evidence.py \
  --scene usd/scenes/warehouse_box_transfer.usda \
  --output-dir evidence/workcell/$(date +%F)/manual --headless

# 动态蓝箱：重力、台面接触、Tag 跟随、reset
env -u PYTHONPATH -u CMAKE_PREFIX_PATH OMNI_KIT_ACCEPT_EULA=YES \
  uv run --frozen python scripts/verify_box_physics_smoke.py \
  --scene usd/scenes/warehouse_box_transfer.usda \
  --output evidence/physics/$(date +%F)/box_gravity_contact_reset.json
```

最新正式场景的 ROS 2 验收必须在 Isaac Sim ROS 2 Bridge 已启动后，从系统 Jazzy 终端启动；验证器只接受时间戳完全相同的 `Image`/`CameraInfo` 对，不能拿历史 JSON 回填：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 scripts/verify_ros2_camera_topics.py \
  --timeout 45 \
  --output evidence/cameras/$(date +%F)/formal_ros2/ros2_topics.json
```

| 阶段 | 预计时间 | 工作内容 | 主要产物与完成门槛 |
|---|---:|---|---|
| P0：汇报基线 | 1～2 天 | Leader 汇报优先；明确已有、规划中、待验证 | PPT v0.2 + PDF；逐页渲染无溢出 |
| P1：环境与资产 | 2～3 天 | subagent 并行锁定 `uv`、Python 3.11、Isaac 5.1.0.0、Lab 2.3.2；预检本地素材 | `uv.lock`、README、预检日志；`uv sync --frozen` 与单测通过 |
| P2：控制与传感底座 | 5～7 天 | URDF/USD/关节映射、ROS 2 Bridge、MoveIt、AprilTag、四相机 | 机器人完整 USD、四相机证据、ROS topic 实订阅、控制冒烟测试 |
| P3：固定底盘搬箱 | 5～6 天 | 双臂预抓、对称夹持、受控附着、抬升、放置、撤离 | 连续完成 20 次；跟踪误差目标 ≤0.01 rad |
| P4：完整移动搬箱 | 5～7 天 | 底盘接近、相机交接、精对准、抓取搬运和故障恢复 | 50 次成功 ≥48 次；停车误差目标 ≤30 mm/3°；2 小时稳定性 |
| P5：手机扫码装箱 | 10～15 天 | 取机、调姿、扫码、PASS/重试/NG、参数化插槽和断点恢复 | 正式模型整箱循环无漏槽/重槽；分支路径测试完整 |
| P6：机床上下料计划 | 3～4 天 | 状态机、导航/操作/PLC/安全架构与资料清单 | 详细技术方案和验收设计；不实现机床代码 |

每阶段均执行以下质量门：

- 代码门：格式/静态检查（引入后）、无 Isaac 单测、必要的 Isaac 集成测试；新增公开接口有 docstring，关键状态、单位和安全逻辑有注释。
- USD 门：`usdchecker`（或 Isaac 对应验证工具）检查、相对引用检查、GUI 打开检查、物理播放检查。
- 可复现门：清理本地生成缓存后，按 README 的命令重新创建挂载并运行对应阶段。
- 证据门：每项集成能力有运行日志和截图/录屏；相机能力必须满足第 4.3 节的四图证据。

## 6. subagent 驱动实施与审查制度

本项目默认采用 **subagent 驱动**，特别是配环境这类简单、可验证、适合并行的任务。主任务不重复手工执行所有机械步骤，而是把任务拆成有明确输入、输出、文件边界和验证命令的工作包；主任务统一集成，不能把 subagent 的“已完成”直接当作验收结论。

```text
主任务定义接口与质量门
   ├─ Terra medium：环境/依赖/资产预检与简单验证
   ├─ Terra medium：边界清楚的代码、测试、文档与证据整理
   └─ Sol ultra：架构、USD、物理/相机证据终审与调度
                 ↓
          主任务复核并集成交付
```

| subagent 工作包 | 默认模型 | 必须输出 | 主任务复核 |
|---|---|---|---|
| **环境配置**：创建 `.venv`、锁版本、配置 NVIDIA 索引、`uv sync --frozen` | `gpt-5.6-terra` medium | 修改文件、完整命令、版本输出、失败日志 | Python/Isaac 版本、锁文件差异、是否修改安装包源码 |
| 本地资产预检：`ISAAC_ASSET_ROOT`、Warehouse、AprilTag、磁盘与相对挂载 | `gpt-5.6-terra` medium | 资产路径清单、预检日志、缺失项 | 随机抽查路径并重新运行预检 |
| Hugging Face 候选筛选：数据卡、许可证、固定 revision、最小样本与格式清单 | `gpt-5.6-terra` medium | 候选表、用途、大小、commit、许可证和拒绝原因 | 核对 Hub API/数据卡，确认未下载大仓库或执行远程代码 |
| ROS/Isaac 冒烟测试：Jazzy、DDS、Bridge、CameraInfo、MoveIt topic | `gpt-5.6-terra` medium | 启动命令、topic/type/frame 输出、日志位置 | 独立订阅一次并比对 frame/topic |
| 场景/代码：资产导入、状态机、执行器、单测 | `gpt-5.6-terra` medium | 边界内代码、测试、说明 | 接口、单位、注释、回归测试 |
| 跨模块审查与调度：USD/URDF/ROS 一致性、架构、物理/相机证据和阶段门 | `gpt-5.6-sol` ultra | 按严重度排序的问题清单、复查结论与下一步调度 | 主任务逐项修复或登记决策 |

执行规则：

1. 每个 subagent 开始前必须声明目标、允许修改的文件和验收命令；避免两个 agent 同时编辑同一 USD 组合层、README 或锁文件。
2. 环境 subagent 不得改写 `.venv` 内 Isaac Sim/Isaac Lab 源码，也不得用 Conda 代替项目 `.venv`；个人绝对路径只作为 shell 环境变量示例，不写入运行代码或 USD。
3. 每个环境或资产工作包都要回传精确版本、命令与日志，失败时保留原始原因，不能用“本机可运行”替代可复现证据。
4. Sol ultra 在 P1、P2、P4 三个门槛进行独立审查；相机、关节映射、ROS topic 和实测指标必须以文件与运行证据为依据。
5. 主任务在合并 subagent 产物后重新运行最小测试集和关键截图；有文件冲突或结论不一致时，以官方 NVIDIA 5.1 文档、实际运行结果和项目验收条件裁决。

## 7. README 与最终交付清单

最终 README 至少应覆盖：支持版本、GPU/驱动前提、`uv` 安装、Isaac Sim/Lab 安装来源、`ISAAC_ASSET_ROOT`、资产预检与挂载、各阶段运行命令、单元/集成测试、打开 USD 的方式、四相机截图命令与证据位置、常见路径问题的排查方式。

最终交付必须包含：

- 可安装的 `source/franzi_sim` 代码、测试和启动/验收脚本。
- 所有项目自有 USD、URDF、导入配置、碰撞层和顶层场景组合；不得遗漏运行所需的项目自有文件。
- 指向官方素材的相对引用和可重复创建的挂载步骤；不包含 NVIDIA 大型素材的拷贝。
- 四相机配置、可获取 RGB 画面的代码、RGB/CameraInfo ROS 2 发布图、四路 GUI/RTX 截图与 manifest。
- 搬箱端到端运行记录或录屏、验收结果，以及以“完成内容、系统结构、实现方式、演示结果”为主的汇报 PPT。
- 手机扫码装箱代码、完整项目自有 USD 和整箱测试证据（P5 完成时）；机床上下料详细计划（P6）。
- README、资产/参考登记、本实施计划和变更记录。

## 8. 实施者注意事项

- 不混用 Isaac Sim 6.x 示例/API 与 5.1 运行时；新增 API 先登记来源版本。
- 统一长度单位为米；STL 导入与质量、关节限位、相机位姿要在导入后复核。
- 相机不能只停留在 URDF frame 或 USD prim：必须连通 render product 并通过 GUI 截图验证。
- 在运行和导入脚本中禁止写死 `/home/fmc3/...` 等个人绝对路径；资产根只从环境变量或参数读取。
- 对双臂抓取和移动底盘的坐标系、关节名、碰撞组变更，同时更新 USD、URDF、配置和测试。

## 9. 更新规则与变更记录

实施者完成一个阶段后，在本文件相应阶段末尾补充日期、实现范围、运行命令、测试结果和证据路径；不要只在聊天或 PPT 中记录结果。影响接口、资产尺寸、相机数量/安装位、版本或验收标准的改动必须新增一行变更记录，并同步 README、`docs/assets.md` 或 `docs/references.md`。

| 日期 | 变更 | 原因 | 影响范围 | 确认人 |
|---|---|---|---|---|
| 2026-07-29 | 创建实施基线：搬箱优先、四相机真实画面验收、18槽手机后续规划 | 已确认交付方向 | 全项目 | 项目负责人 |
| 2026-07-29 | subagent 驱动升级为正式执行制度，环境配置优先由 Terra medium 并行完成，Sol ultra 审查阶段门 | 负责人明确要求 | 环境、实现、测试与验收 | 项目负责人 |
| 2026-07-29 | 正式场景升级为多工位 Warehouse，蓝箱附 Tag 0；物理 smoke 和正式 RTX 证据归档 | 先建立可见、可验证的搬箱工位基线 | USD、相机、物理、PPT | 项目负责人 |
| 2026-07-30 | 主 PickTable 对称桌角新增独立 `tag36h11` Tag 1/2；与随箱 Tag 0 分层、分 prim、分验证角色 | 为工位坐标提供不随任务对象运动的静态定位基准 | USD、静态验证、README/资产约定 | 项目负责人 |
| 2026-07-29 | 将 Hugging Face 固定版本数据加入第三层参考源，建立最小下载、许可证与兼容性门 | 负责人补充可用的 Isaac Lab/Isaac Sim 数据来源 | 资料、资产、数据结构与 subagent 调研 | 项目负责人 |
| 2026-07-30 | D405 实体镜头、USD 光轴和夹爪 `-Z` 同轴；自然下垂时朝地，USD/ROS roll 分别为 `Rz90` / `Rz90·Rx180`（先安装旋转，再乘局部 optical 变换）；中立腕相机不设 Tag 解码门。机器人根节点按轮面抬高 87.3 mm，并显式审计 37 个 convex hull | 以夹爪物理方向和真实 D405 参考画面重新定义验收 | 相机、URDF/USD、TF、场景、碰撞、测试与证据 | 项目负责人 |
