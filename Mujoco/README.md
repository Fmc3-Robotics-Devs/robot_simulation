# MuJoCo 阶段

用 SolidWorks 导出的 `wheel_robot_26.8.16_3` 建的 Franzi MuJoCo 物理仿真包。
与 Isaac 阶段"只做传感器与世界、镜像 ROS 栈"不同,这里**跑的是物理**:
关节由伺服执行器驱动、带重力,工件是自由刚体,抓取靠手指摩擦力而不是
attach。控制接口用机器人自己的语言(SRDF 命名姿态、夹爪开口、底盘 twist、
TCP 位姿),现在由 Python 脚本驱动,ROS 桥接留作下一步。

```text
wheel_robot_26.8.16_3/   SolidWorks 原始导出(输入,不改)
convert_urdf.py          URDF → model/franzi.xml(机器人) + model/scene.xml(场景)
franzi.py                Python 接口:姿态 / 夹爪 / 底盘 / IK / 相机 / 2D 雷达
demo.py                  取放演示:行驶 → 停靠 → 抓取 → 平移 → 放置
check_model.py           自检(每一条设计声明对应一项检查)
```

## 安装与运行

从仓库根目录执行。用系统 Python 3.12 建 venv(与 ROS 2 Jazzy 的 rclpy 同版本,
将来接 ROS 可直接用;不要用 conda 的 python):

```bash
/usr/bin/python3 -m venv Mujoco/.venv
Mujoco/.venv/bin/pip install -r Mujoco/requirements.txt

Mujoco/.venv/bin/python Mujoco/convert_urdf.py          # 生成 model/(改了 URDF/SRDF/task.yaml 后重跑)
Mujoco/.venv/bin/python Mujoco/demo.py                  # MuJoCo 窗口里看取放演示
Mujoco/.venv/bin/python -m mujoco.viewer --mjcf Mujoco/model/scene.xml   # 只看模型,可手动拖动/调关键帧
```

也可以不建新环境,直接复用 Isaac 的 `env_isaaclab_6`:它自带 mujoco 3.10.0、numpy、
pyyaml,本包无需安装任何东西即可运行(`check_model.py` 全部通过)。**但不要往这个环境里
`pip install -r requirements.txt` 或升级 mujoco**——Isaac Lab 3 的 Newton 后端锁定了
`mujoco~=3.10.0` 和 `mujoco-warp`,升级会弄坏 Isaac。

```bash
PYTHONNOUSERSITE=1 conda run --no-capture-output -n env_isaaclab_6 python Mujoco/demo.py
```

MuJoCo 与 Isaac Sim 不在同一进程、不共享数据(Isaac 用的是 PhysX),两边的 MuJoCo 版本
不需要一致;只有装在同一个环境里时才受 Isaac 的版本锁约束。

无显示器时渲染用 EGL:

```bash
MUJOCO_GL=egl Mujoco/.venv/bin/python Mujoco/demo.py --record Mujoco/renders/demo.mp4
MUJOCO_GL=egl Mujoco/.venv/bin/python Mujoco/demo.py --record Mujoco/renders/head.mp4 --camera head_d435
```

录像直接通过 ffmpeg 编成 H.264 / yuv420p / faststart,常见播放器都能打开。

## 验证

```bash
MUJOCO_GL=egl Mujoco/.venv/bin/python Mujoco/check_model.py
```

2026-09 在本机实测 24 项全部通过(mujoco 3.13.0 / 3.10.0 / 3.8.1 各跑一遍),约 2.5 s:

| 检查 | 对照 | 结果 |
|---|---|---|
| 原始导出与 `franzi_description` 两种输入 | 生成的 MJCF 逐字节比较 | 完全一致 |
| 正运动学 | `franzi_description` URDF,46 个 link/工具帧 × 10 组随机姿态 | 误差 1.4e-8 m |
| 轮子触地高度 / 轮半径 | `task.yaml` 的 `ground_z` / `wheel_radius` | -0.0873 / 0.0827,一致 |
| 各关键帧静止保持 3 s | 关节 effort 上限 | 最大负载 21%,漂移 0 |
| 夹爪 | SRDF open/closed | 张开 95.0 mm,闭合 0.0 mm(左右) |
| 底盘 | 横移 / 原地转 / `drive_to` | 舵轮 90° / 切向,落点 0.8 mm |
| 底盘顶到桌子后停车 | anti-windup | 不回冲(0 mm) |
| 相机 | 世界方向 | D435 画面正立;腕部 D405 手指在画面下半 |
| 2D 雷达 | 桌腿位置 | 819 束,最近回波 0.540 m |
| 取放(纯物理) | 目标位置 | 夹住 69.8 mm、抬起 119 mm、横移中离 TCP 4.6 mm、落点误差 0.3 mm |

## 模型的关键取舍(为什么这样做)

1. **坐标修正与 `franzi_description` 同一套。** 导出件的根坐标系让机器人沿 +X 站立,
   不是 REP-103。转换器发现根 link 带底盘网格时,把它改名 `base_body_Link`,前面加一个
   无质量的 `base_link`,变换与 `franzi_description` 追加的修正块完全相同。于是
   **MuJoCo 世界系就是 ROS 的 `odom` 系**,地面在 `ground_z = -0.0873`,所有位姿可直接
   与 ROS 栈比对。输入也可以换成已修正的 `franzi_description` URDF
   (`--urdf Rviz/src/franzi_description/urdf/wheel_robot_26.8.16_3.urdf`),
   两者产物逐字节相同。SDK 的 `*_flange` / `*_tcp` 帧以 site 形式保留。
2. **底盘是全向平面关节,不是轮地接触。** 与 SRDF 的 planar `world_joint` 一致:
   `base_link` 挂在 x / y / yaw 三个关节上,用积分速度执行器(`intvelocity`)驱动,
   速度为零时原地保持位置。三个舵轮模组的转向角和轮速按运动学跟随(与
   `franzi_pick_place/base.py` 同一套逻辑:走短弧、必要时反转轮速),但轮子不参与碰撞,
   三个凸包轮压在地面上只会与平面关节打架。底盘被挡住时积分目标被限制在实际位置
   5 cm / 0.1 rad 之内,脱困后不会冲出去。
3. **碰撞几何 = 各 link 网格的凸包,只与环境碰撞,不做自碰撞。** 相邻 link 的凸包在真实
   网格不重叠处也会重叠,开自碰撞会卡死关节;自碰撞是 MoveIt 规划层的职责。手指用高摩擦
   (μ=1.5)+ 扭转摩擦,工件用铝的密度(0.53 kg)。
4. **执行器。** 位置伺服的增益按 `home` 姿态下的关节等效惯量定(身体约 4 Hz、手臂 15–20 Hz,
   接近临界阻尼)。所有 link 开 `gravcomp`,并用 `actuatorgravcomp` 把重力补偿计入执行器
   力矩——相当于真机控制器的重力前馈,补偿 + 伺服合计仍受 effort 上限约束。
   夹爪:每只手一个伺服驱动 finger01,finger02 由等式约束镜像,力上限 100 N。
5. **⚠ effort 是假设值。** 导出件里所有关节都是 `effort="100" velocity="3.14"`(导出器
   占位值,右手手指甚至是 0/0)。手臂 100 N·m 足够(工作空间内最坏静负载约 25 N·m),
   但手臂前伸时 thigh 关节单是撑住上身就需要约 110 N·m,所以 calf / thigh / waist_pitch
   三个身体关节**假设为 400 N·m**(`convert_urdf.py` 的 `BODY_EFFORT`)。拿到真实电机参数
   后替换。2Dlidar_Link 导出的惯量全零,换成同质量 3 cm 实心球。
6. **相机。** 四个相机都挂在 URDF 相机 link 上,光学参数用 Isaac 的 D435 / D405 数值。
   **注意:这次导出把相机 link 的朝向改了。** 旧模型 `wheel_robot_4.0` 的光学帧是
   `link·Rz(-90°)`,新模型上 D435 需要 `link·Rz(180°)` 画面才是正的(`check_model.py`
   验证了 image-right = 机器人右侧、image-down = 世界向下)。腕部 D405 取 `Rz(+90°)`,
   手指出现在画面下方;真机 D405 的安装方向还需确认。**ROS 侧
   `tending_cell.launch.py` 里的 `head_d435_optical` 静态 TF(`--yaw -1.5707963`)对新模型
   已经不对**,迁移 `franzi_pick_place` 时需要一起改。
   头部 D435 镜头前 2–3 cm 就是面罩外壳(真机那里有透光窗),所以近裁剪面设为 5 cm
   (与 Isaac 的 clipping_range 一致),否则会拍到面罩内壁。
7. **场景来自 `task.yaml`。** 地面高度、桌子尺寸/高度、工件尺寸、停靠偏置都从
   `Rviz/src/franzi_pick_place/config/task.yaml` 读;SRDF 的命名姿态组合成关键帧
   `home`(work 身体 + 手臂下垂 + 夹爪张开)、`look_down`、`zero`。代码里不写死位姿。

## Python 接口速览

```python
from franzi import Franzi, top_down
robot = Franzi()                                # 加载 scene.xml,复位到 "home"
robot.set_posture("head", "look_down")          # SRDF 命名姿态
robot.drive_to(0.0, 0.25, 0.0)                  # odom 位姿,底盘 + 舵轮跟随
robot.drive(0.2, 0.0, 0.3); robot.step(1.0)     # 机体系 twist,推进 1 s 仿真时间
q, ok = robot.solve_ik("left", [0.44, 0.16, 0.95], top_down())   # TCP 竖直向下
robot.move_joints(q, 2.0)                       # 最小加加速度插值
robot.move_tcp("left", [0.44, 0.16, 0.85], top_down(), 1.5)      # 直线,拒绝换 IK 分支
robot.set_gripper("left", 0.0)                  # 开口(m),0 = 闭合
rgb = robot.render("head_d435"); depth = robot.render("head_d435", depth=True)
angles, ranges = robot.scan()                   # 2Dlidar,270°/0.33°,只看环境
```

所有动作都经执行器施加,不直接改关节位置(`place_base` 例外,仅用于初始摆放)。

## 已知限制 / 下一步

- 没有 ROS 桥接。自然的下一步:发布 `/joint_states`、`odom→moveit_root` TF、相机与 `/scan`,
  订阅 FollowJointTrajectory / `cmd_vel` / 夹爪命令,使 `tending_cell.launch.py` 可以
  `backend:=mujoco`。venv 基于系统 Python 3.12 就是为此准备的。
- 场景只有一张桌子 + 一块工件(停靠姿态下的相对布局),没有搬完整的三工位车间、雕刻机和
  AprilTag;抓取用的是工件真值位姿(相当于 mock 检测)。
- 底盘是平面关节而非轮地接触,打滑、颠簸等不在模型里。
- 碰撞用凸包,手指之间、夹爪掌心内侧的精细几何被凸包"填平"。
