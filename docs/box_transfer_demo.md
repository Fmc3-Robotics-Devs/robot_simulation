# Wheel Bot 搬箱软件在环演示

## 1. 用途与边界

本演示在正式场景 `usd/scenes/warehouse_box_transfer.usda` 中执行一条可重复的 21 秒全身关键帧轨迹，用于验证：

- Wheel Bot 底盘接近、转向轮/轮子、头部、双臂和夹爪动作能在同一 articulation 中连续执行；
- 蓝色运输箱能够从 PickTable 一角被抬起、横向搬运并放到另一角；
- 箱上 AprilTag 0 随箱移动，桌角 AprilTag 1/2 保持静态；
- 总览、头部 D435、胸部 D435、左腕 D405、右腕 D405 五路 RGB 能在同一仿真时刻取帧并生成视频；
- 动作、场景、源码、逐帧状态和验收指标能形成可追溯证据。

这是 **controlled-attachment software-in-the-loop** 演示。机器人根位姿与关节采用确定性关键帧；蓝箱保持为动态刚体，但在双手抓取门槛通过后，每个 120 Hz 周期把它约束到双夹爪中点，释放后停止约束并交还 PhysX。该结果不能替代 MoveIt/ROS 2 闭环、力控或纯摩擦抓取验收。

## 2. 动作时序

蓝箱起点为 `(1.20, 0.38, 0.995) m`，目标桌角为 `(1.20, -0.72, 0.995) m`。

| 时间 | 阶段 | 动作 |
|---:|---|---|
| 0.00–0.75 s | `home` | 全零中立姿态，四相机建立初始帧 |
| 0.75–3.25 s | `approach` | 底盘接近取料桌，头部下俯观察 |
| 3.25–5.75 s | `pregrasp` | 双臂镜像展开，夹爪从箱体两侧对准 |
| 5.75–6.75 s | `grasp` | 四指对称闭合 |
| 6.75–9.00 s | `lift` | 双臂抬升约 280 mm，底盘后退约 100 mm 保持箱体前后位置 |
| 9.00–12.00 s | `transport` | 底盘沿桌面长边横移到另一角 |
| 12.00–14.25 s | `lower` | 双臂下降，箱体回到桌面高度 |
| 14.25–15.25 s | `release` | 张开夹爪；完全张开前保持箱体，避免闭合夹指在解除瞬间顶推箱体 |
| 15.25–16.50 s | `withdraw` | 解除受控约束；保持双臂张开姿态，底盘直线后退，使夹指先离开箱体 |
| 16.50–17.75 s | `retreat` | 脱离箱体后收回双臂，底盘进入复位起点 |
| 17.75–20.25 s | `return_home` | 底盘与全身关节复位 |
| 20.25–21.00 s | `complete` | 保持完成画面 |

所有过渡使用五次多项式插值，端点速度和加速度为零。轮子累计转角按 `2π` 周期回绕，避免 PhysX 连续转动关节拒绝超范围 drive target。

## 3. 抓取与释放门槛

受控附着只能在以下条件同时满足时启用：

1. 左右夹爪测量点到箱体两侧预期抓取点的最大误差不超过 `30 mm`；
2. 左右 wrist-roll/夹爪 `-Z` 与箱体前向 `+X` 的点积均不小于 `0.995`；
3. 四个夹指达到 `±32 mm` 闭合命令，最大命令误差不超过 `1 µm`；
4. 合爪前箱体相对正式起点漂移不超过 `20 mm`。

运输阶段箱底相对桌面至少抬高 `250 mm`。释放后额外运行一秒 PhysX 接触，再检查：

- 目标桌角 XY 偏差不超过 `30 mm`；
- 箱体根高度与桌面高度差不超过 `3 mm`；
- 姿态偏差不超过 `2°`；
- 线速度和角速度范数均不超过 `0.02`；
- 全身关节设置误差不超过 `0.01 rad`。

30 mm 是当前软件在环放置门，不应在 PPT 中写成真实机器人生产精度。正式运行的实际数值只从 `manifest.json` 读取。

## 4. 五路同步录像

每个视频时刻先由 120 Hz 控制循环推进物理，再使用 NVIDIA Replicator `orchestrator.step(delta_time=0.0)` 同步读取五个 render product。取帧不会额外推进仿真，避免机器人在相邻源帧之间出现未受控的物理步。

| 视图 | 来源 | 分辨率 | 作用 |
|---|---|---:|---|
| `overview` | 任务级独立 RTX 相机 | 1280×720 | 检查机器人、桌面、箱体和完整路径 |
| `head` | `head_d435_Link` | 1280×720 | 正面观察箱体与远端工位 |
| `chest` | `body_d435_Link` | 1280×720 | 近场桌面/箱体下缘视角 |
| `left` | `left_wrist_d405_Link` | 1280×720 | 沿左夹爪 `-Z` 观察 |
| `right` | `right_wrist_d405_Link` | 1280×720 | 沿右夹爪 `-Z` 观察 |

D405 是刚性安装，画面 roll 会随 wrist-roll 关节变化；验收重点是“夹爪指向哪里，相机就看向哪里”，而不是用数字后处理把地平线强行转正。原始五路帧不会旋转或裁切；只有合成板按固定布局缩放。

## 5. 运行与输出

```bash
demo_output=/tmp/wheelbot_box_transfer
env -u PYTHONPATH -u CMAKE_PREFIX_PATH OMNI_KIT_ACCEPT_EULA=YES \
  uv run --frozen python scripts/record_box_transfer_demo.py \
  --scene usd/scenes/warehouse_box_transfer.usda \
  --output-dir "$demo_output" \
  --fps 15 --rt-subframes 4 --headless
```

输出目录必须不存在或为空。成功结果包含：

```text
manifest.json
timeline.json
thumbnail.png
videos/
├── wheelbot_box_transfer_multiview.mp4
├── overview.mp4
├── head.mp4
├── chest.mp4
├── left.mp4
└── right.mp4
keyframes/
└── 代表各主要阶段的 1920×1080 合成 PNG
frames/
└── 五路逐帧 PNG（正式 Git 归档可省略）
```

`manifest.json` 记录正式场景哈希、源码 bundle 哈希、Git 状态、Isaac Sim 版本、帧率、五路像素统计、抓取门、附着误差、关节误差、放置误差和全部运行时检查。`ok: true` 只代表本文件定义的软件在环门槛通过。

## 6. 视觉复核

正式录制后必须人工查看至少以下画面：

1. `home`：机器人轮面不陷地、箱体位于起点、Tag 0/1/2 布局正确；
2. `pregrasp`：双臂分别位于箱体两侧，未发生明显穿桌；
3. `lift`：箱体离开桌面，头部与双腕均获得非空图像；
4. `transport`：机器人直立，箱体随底盘横移；
5. `lower/release`：箱体进入目标角并由夹爪释放；
6. `complete`：机器人复位，箱体留在另一桌角；
7. 五路视频无黑帧、旧帧混入、帧名错位或左右腕交叉映射。

项目已使用 headless RTX 原始帧和合成关键帧进行截图复核；正式证据归档在 `evidence/box_transfer/2026-07-30/final/`。
