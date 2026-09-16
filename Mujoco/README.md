# MuJoCo 阶段

Franzi 的 MuJoCo 物理仿真包,模型来自两份 SolidWorks 导出的合并:底盘到肘部取
`wheel_robot_26.8.16_3`,手腕与夹爪取 `wheel_robot_7.24`(见下文"合并参考 URDF")。
与 Isaac 阶段"只做传感器与世界、镜像 ROS 栈"不同,这里**跑的是物理**:
关节由伺服执行器驱动、带重力,工件是自由刚体,抓取靠手指摩擦力而不是
attach。控制接口用机器人自己的语言(SRDF 命名姿态、夹爪开口、底盘 twist、
抓取中心位姿),现在由 Python 脚本驱动,ROS 桥接留作下一步。

```text
wheel_robot_26.8.16_3/   SolidWorks 导出:底盘、腿、躯干、头、肩、肘(输入,不改)
wheel_robot_7.24/        SolidWorks 导出(即旧 wheel_robot_4.0):手腕、夹爪、腕部 D405(输入,不改)
merge_urdf.py            两份导出 → franzi_merged/(MuJoCo 的参考 URDF + 用到的网格)
franzi_merged/           合并结果,已提交;改了导出后重跑 merge_urdf.py,不要手改
convert_urdf.py          franzi_merged URDF → model/franzi.xml(机器人) + model/scene.xml(场景)
pem_cell.yaml            PEM 叠片工位布局(测量图纸的数值,mm)
pem_cell.py              pem_cell.yaml → model/pem_scene.xml(PEM 工位场景,见下文)
pem_demo.py              PEM 搬运演示:用叉式夹具把 4 叠极片从边桌送进阳极/阴极 tray
PEM Project/             工位测量图、阳极/阴极片图纸、tray.stl、参考视频(输入,不改)
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

Mujoco/.venv/bin/python Mujoco/merge_urdf.py            # 重建 franzi_merged/(只在换了导出后需要)
Mujoco/.venv/bin/python Mujoco/convert_urdf.py          # 生成 model/(改了 URDF/SRDF/task.yaml 后重跑)
Mujoco/.venv/bin/python Mujoco/demo.py                  # MuJoCo 窗口里看取放演示
Mujoco/.venv/bin/python -m mujoco.viewer --mjcf Mujoco/model/scene.xml   # 只看模型,可手动拖动/调关键帧

Mujoco/.venv/bin/python Mujoco/pem_cell.py              # 生成 model/pem_scene.xml(在 convert_urdf.py 之后;改了 pem_cell.yaml 后重跑)
Mujoco/.venv/bin/python -m mujoco.viewer --mjcf Mujoco/model/pem_scene.xml   # 看 PEM 工位场景
Mujoco/.venv/bin/python Mujoco/pem_demo.py              # MuJoCo 窗口里看 PEM 搬运(约 225 s 仿真时间)
MUJOCO_GL=egl Mujoco/.venv/bin/python Mujoco/pem_demo.py --record Mujoco/renders/pem.mp4 [--camera anode_tray_view]
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

先跑过 `convert_urdf.py` 和 `pem_cell.py`。2026-09-14 在本机实测 37 项全部通过
(mujoco 3.13.0 与 3.10.0 各跑一遍),约 55 s(其中 PEM 搬运演示约 40 s):

| 检查 | 对照 | 结果 |
|---|---|---|
| 合并产物是最新的 | 重跑 `merge_urdf.py` 与磁盘上的 URDF / 39 个网格逐字节比较 | 一致 |
| 底盘到肘部 + SDK flange/TCP | `franzi_description` URDF,32 个帧 × 10 组随机姿态 | 误差 0.002 mm |
| 手腕与夹爪 | `wheel_robot_7.24` URDF,12 个帧 × 10 组随机姿态 | 误差 0.008 mm |
| 抓取中心 `*_grasp` | `task.yaml` 的 `tcp_offset`(旧模型 wrist_roll_Link 系) | 差 0.14 mm |
| 正运动学 | 合并 URDF,46 个 link/工具帧 × 10 组随机姿态 | 误差 1.2e-8 m |
| 轮子触地高度 / 轮半径 | `task.yaml` 的 `ground_z` / `wheel_radius` | -0.0873 / 0.0827,一致 |
| 各关键帧静止保持 3 s | 关节 effort 上限 | 最大负载 21%,漂移 0 |
| 夹爪 | 开口定义 | 张开 95.0 mm,闭合 0.0 mm(左右) |
| 底盘 | 横移 / 原地转 / `drive_to` | 舵轮 90° / 切向,落点 0.8 mm |
| 底盘顶到桌子后停车 | anti-windup | 不回冲(0 mm) |
| 相机 | 世界方向 | D435 画面正立;腕部 D405 手指在画面下沿 |
| 2D 雷达 | 桌腿位置 | 819 束,最近回波 0.540 m |
| 取放(纯物理) | 目标位置 | 夹住 69.8 mm、抬起 118 mm、横移中手内滑移 0.4 mm、落点误差 1.4 mm |
| PEM:三个 tray 的 x 间距 | 测量图 1175 / 420 / 550 mm | 一致 |
| PEM:tray 离地高度 | 测量图 1061 / 1061 / 956 mm | 一致 |
| PEM:装配 tray | 1175 − 420 − 550 = 205 宽、192 深,中心线靠后 27 mm | 一致 |
| PEM:型材 | 50 × 50,立柱在各 tray 下(x 居中)、顶到 tray 底 | 一致 |
| PEM:极片叠 | 图纸尺寸 × 40 片 | 一致;阳极 0.61 kg,阴极 1.10 kg,夹具 0.58 kg |
| PEM:tray 导向板 / 夹具 | 两种片子本体;短边角柱间的缺口 | 留 183.5 × 119.5 mm;夹具 40 mm 宽过 47.5 mm 缺口 |
| PEM:静置 | 物理 | 起始 0 穿透,2 s 沉降 0.23 mm,夹具压紧力 40 N |
| PEM:搬运演示(纯物理) | 4 叠各进各的 tray、两层 | 都在导向板内,层高误差 0.06 mm,倾斜 ≤ 0.03°;夹具放回原位 ≤ 0.8 mm;机器人除手柄外无任何接触 |

## 模型的关键取舍(为什么这样做)

1. **合并参考 URDF(`merge_urdf.py`)。** 两份导出单独都不对:`wheel_robot_26.8.16_3`
   从 wrist_yaw 关节往下与真机不符(夹爪绕工具轴转了 90°、腕部 D405 位置不同、
   wrist_yaw / wrist_pitch 正方向相反),`wheel_robot_7.24` 的这一段是对的。两份导出
   在零位时从底盘到 wrist_pitch_Link 的网格重合到 0.005 mm 以内,各手臂关节轴也是同一
   条直线,所以合并是精确的:以 26.8.16_3 为主体,把每侧 `{side}_wrist_yaw_Link` 子树
   整个换成 7.24 的——关节、坐标系、限位、惯量、网格原样照搬。只有三处是新加的:
   - **嫁接变换。** 两份导出给 elbow_pitch_Link 定的坐标系不同,`wrist_yaw_joint` 的
     原点用零位正运动学换算到 26.8.16_3 的 elbow 系:xyz (0.017, 0.1262, 0)、
     rpy (-π/2, 0, π)。26.8.16_3 把 π/2 写成 1.5708,换算结果带 µm 级残差;脚本把它
     吸附到设计值,残差超过 0.05 mm / 1e-4 就报错停下。
   - **命名。** 沿用 MoveIt 配置里的现名:`leftfinger1/2` → `left_finger01/02`,
     `left_wrist_d405` → `left_D405`(右侧同理)。**关节正方向保持 7.24 的**,因此与
     ROS 侧 `franzi_description` 相比,**wrist_yaw、wrist_pitch 转向相反**,手指关节的
     行程符号也不同(两侧 finger01 都是 [0, 0.0475]);`check_model.py` 会逐项列出。
     将来接 ROS 桥时要按关节做符号映射。
   - **坐标系。** REP-103 根(与 `franzi_description` 同一套:导出件让机器人沿 +X 站立,
     根 link 改名 `base_body_Link`,前面加无质量的 `base_link`),于是 **MuJoCo 世界系就是
     ROS 的 `odom` 系**,地面在 `ground_z = -0.0873`,所有位姿可直接与 ROS 栈比对。
     再加三组工具帧(转成 site):`*_flange` 是 SDK 法兰(J7 腕 roll 轴中心),与
     `franzi_description` 的是同一个物理帧;`*_tcp` 是 SDK TCP,法兰 +x 方向 273.5 mm、
     左右各绕 x 转 ∓90°——它是手工给 SDK 定的,与夹爪无关,在这把夹爪上落在指尖下方
     15 mm;`*_grasp` 是张开时两指指腹的中心,+x 为接近方向、+z 为 finger01 的闭合方向,
     在 wrist_roll_Link 下为 (-0.0304, ∓0.0017, -0.2414),即 ROS 侧 `task.yaml` 的
     `tcp_offset`。IK 与直线运动默认控制 `*_grasp`。
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
   夹爪:每只手一个伺服驱动 finger01,finger02 由等式约束镜像,力上限 100 N;闭合
   方向由 finger01 的限位读出,不写死。
5. **⚠ effort 是假设值。** 导出件里所有关节都是 `effort="100" velocity="3.14"`(导出器
   占位值)。手臂 100 N·m 足够(工作空间内最坏静负载约 25 N·m),
   但手臂前伸时 thigh 关节单是撑住上身就需要约 110 N·m,所以 calf / thigh / waist_pitch
   三个身体关节**假设为 400 N·m**(`convert_urdf.py` 的 `BODY_EFFORT`)。拿到真实电机参数
   后替换。2Dlidar_Link 导出的惯量全零,换成同质量 3 cm 实心球。
6. **相机。** 四个相机都挂在 URDF 相机 link 上,光学参数用 Isaac 的 D435 / D405 数值。
   **注意:26.8.16_3 导出把 D435 相机 link 的朝向改了。** 旧模型 `wheel_robot_4.0` 的
   光学帧是 `link·Rz(-90°)`,新模型上 D435 需要 `link·Rz(180°)` 画面才是正的
   (`check_model.py` 验证了 image-right = 机器人右侧、image-down = 世界向下)。腕部
   D405 来自 7.24,挂在夹爪侧面、朝下看;两侧导出的朝向相同但位置镜像,所以左侧取
   `link` 本身、右侧取 `Rz(180°)`,张开的两指都出现在画面下沿的两个角上。真机 D405 的
   安装方向还需确认。**ROS 侧
   `tending_cell.launch.py` 里的 `head_d435_optical` 静态 TF(`--yaw -1.5707963`)对新模型
   已经不对**,迁移 `franzi_pick_place` 时需要一起改。
   头部 D435 镜头前 2–3 cm 就是面罩外壳(真机那里有透光窗),所以近裁剪面设为 5 cm
   (与 Isaac 的 clipping_range 一致),否则会拍到面罩内壁。
7. **场景来自 `task.yaml`。** 地面高度、桌子尺寸/高度、工件尺寸、停靠偏置都从
   `Rviz/src/franzi_pick_place/config/task.yaml` 读;SRDF 的命名姿态组合成关键帧
   `home`(work 身体 + 手臂下垂 + 夹爪张开)、`look_down`、`zero`。代码里不写死位姿。
   SRDF 是 ROS 侧按 `franzi_description` 写的;某个状态值超出本模型关节范围时(例如
   左手的 `closed`,因为手指符号不同)转换器和 `set_posture` 直接报错,不会静默截断。
8. **演示的接近路径。** 关节空间插值不做碰撞检查:从下垂的手臂直接插值到零件正上方,
   手会从桌沿下面扫过去。所以 `demo.py` 先让手在桌沿外侧 20 cm 升到接近高度,再水平
   直线移到零件上方、竖直下降,放下后原路退出;四个抓取 yaw 各规划一遍整条路径,选离
   关节限位最远的可行方案。掌心的碰撞凸包止于 grasp 帧上方 8 mm、指尖在其下方 17 mm,
   抓取时 grasp 帧放在零件顶面下 5 mm。

## PEM 叠片工位场景(`pem_cell.py` / `pem_demo.py`)

按 `PEM Project/20260906 Measurements.pdf` 搭的工位,加一张边桌,桌上是装在叉式夹具里的
极片叠。所有数值在 `pem_cell.yaml` 里(图纸上的数原样用 mm,测量值标 MEASURED,其余是
假设,换成实测值后重跑 `pem_cell.py`),代码里不写死位置。

- **框架。** 50 × 50 铝型材:地面一根横梁,每个 tray 下一根立柱(x 方向居中),立柱顶到
  tray 底。T 形槽画成深色细线(只是显示)。
- **tray。** 阳极、阴极 tray 复用 `tray.stl`(205 × 137 × 45 mm:底板 + 四个 L 形导向
  角柱),长边沿 x。高度 1061 / 1061 / 956 mm 按"地面到 tray 底面"处理(图上没说量的是
  哪一点,底板顶面比底面高 3 mm)。
- **中间(装配)tray。** x 方向三个间距 1175 / 420 / 550 严格保留,中间 tray 的宽度由它们
  推出:1175 − 420 − 550 = 205 mm(测量图注释写的 200 不用);深 192 mm(注释的 t);中心线
  比阳极、阴极 tray 靠后 27 mm(俯视图里三个 tray 不在一条线上)。画成一块 10 mm 厚的平板
  (厚度是假设)。它的立柱仍在横梁上,所以托板相对立柱向后悬出 27 mm。
- **极片叠。** 每叠 40 片(阳极 182 × 119 本体 + 65 × 30 极耳,厚 0.41 × 40 = 16.4 mm;
  阴极 178 × 115,0.42 × 40 = 16.8 mm),极耳朝工位后方,铜色阳极、银色阴极。两叠放进一个
  tray 是 33 mm,在导向角柱 38 mm 以内。密度按涂层箔片估(阳极 1.7、阴极 3.2 g/cm³)。
- **叉式夹具(可开合的包裹结构)。** 每叠一个,蓝色:底下一根 40 × 3 mm 的叉板托住极片叠,
  上面一块压板压住(夹具自带执行器 `<carrier>_clamp`,40 N),+x 端的立柱连到上方的横臂,
  横臂上是给夹爪抓的手柄(20 mm 宽,夹爪夹持力约 75 N)。手柄由生成器放在"夹具 + 极片叠"
  的合重心正上方,抓起来不歪。叉板、压板、横臂都是 40 mm 宽,正好穿过 tray 短边两个角柱
  之间 47.5 mm 的缺口。叉板是低摩擦面(μ 0.15,相当于 PTFE 涂层),要能从极片叠底下抽出。
- **搬运流程(`pem_demo.py`)。** 每个夹具:开到桌边 → 从上方抓手柄 → 举到 tray 高度以上
  → 开到 tray 前 → 从 tray 的 +x 侧 210 mm 处、在角柱上方沿 −x 平移进来 → 竖直下降到叉板
  贴底(第二叠落在第一叠上)→ 压板松开 → 提起 2 mm,沿 +x 把叉板从极片叠底下抽出、从短边
  缺口出去 → 空夹具放回桌上原位。先两叠阳极,再两叠阴极。全程是物理:夹爪靠摩擦抓手柄,
  极片叠是自由刚体,叉板抽出时靠 +x 端的导向板挡住它。
- **为什么要闭环下降。** 阳极叠在 tray 里单边只有 0.25 mm 间隙。IK 本身有 0.5 mm 容差,
  手臂带着 1.2 kg 负载也会下垂,开环必然压到导向板。所以下降时每 50 ms 量一次极片叠的
  实际 xy,把误差的 30% 加进手的指令(积分;增益再高会因手臂滞后而振荡),落位误差约
  0.02 mm。整条进出路径在夹具移动前先规划成同一个 IK 分支上的一串直线;闭环的每一步都以
  这条路径上的关节为种子——7 自由度手臂有冗余,换了分支,抽出时腕关节会顶到限位。
- **⚠ tray 碰撞比网格宽 0.25 mm。** `tray.stl` 的导向面围出的正好是阳极片本体的
  119 mm。对会变形、有冲切公差的真实极片没问题,对刚体却是过盈配合(偏 0.06° 就卡住),
  所以碰撞导向板比网格往外退了 0.25 mm(`TRAY_PLAY`)。
- **停靠。** `<工位>_dock` site 是去该工位时 base_link 停的位置:tray 前 320 mm、tray 在
  左侧 200 mm(整条进出路径离关节限位最远的位置,底盘离地梁 53 mm);桌子前 350 mm、左侧
  160 mm。换工位时先退到停靠线后 10 cm 的通道上横移,再前进停靠;开车时手一直举在身前
  1.12 m 高,夹具底面高过 tray 顶面。
- 机器人起点在世界原点,面向工位(工位 +x 在机器人右手边,阳极 tray 在左)。场景里有
  `overview`、`anode_tray_view`、`cathode_tray_view` 三个相机。

```python
from franzi import Franzi
import pem_demo
robot = Franzi("Mujoco/model/pem_scene.xml")
robot.set_targets({"anode_carrier_1_clamp": 0.02})   # 打开压板(-0.01 = 压紧)
pem_demo.run(robot)                                    # 或整套搬运
```

**还没做 / 要注意的:**
- 装配 tray 只是平板,叠片(从阳极/阴极 tray 取片叠到装配 tray 上)还没做。
- 夹具、压板执行器、边桌、每叠片数都是假设的设计,换成真实的再改 `pem_cell.yaml`。
- 测量图上横梁上方那条浅色的板/盖没有尺寸,没建。
- 夹爪的夹持力来自位置伺服(闭到 20 mm 手柄时约 75 N),不是力控。

## Python 接口速览

```python
from franzi import Franzi, top_down
robot = Franzi()                                # 加载 scene.xml,复位到 "home"
robot.set_posture("head", "look_down")          # SRDF 命名姿态
robot.drive_to(0.0, 0.25, 0.0)                  # odom 位姿,底盘 + 舵轮跟随
robot.drive(0.2, 0.0, 0.3); robot.step(1.0)     # 机体系 twist,推进 1 s 仿真时间
q, ok = robot.solve_ik("left", [0.44, 0.16, 0.95], top_down())   # left_grasp 竖直向下
robot.move_joints(q, 2.0)                       # 最小加加速度插值
robot.move_tcp("left", [0.44, 0.16, 0.85], top_down(), 1.5)      # 直线,拒绝换 IK 分支
robot.tcp_pose("left", frame="tcp")             # 工具帧默认 grasp;frame="tcp" 为 SDK TCP
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
- 碰撞用凸包,手指之间、夹爪掌心内侧的精细几何被凸包"填平"(7.24 的 wrist_roll 网格
  含夹爪本体,两侧滑块把凸包撑到指根,可用指深约 25 mm)。
- ROS 侧还是 26.8.16_3 的手腕:`franzi_description` / MoveIt 的 wrist_yaw、wrist_pitch
  正方向与这里相反,夹爪几何也不同;而 `franzi_pick_place/config/task.yaml` 仍用旧
  4.0 的名字(`leftfinger1_joint`、`left_wrist_d405_Link`)和 wrist_roll 坐标系下的
  `tcp_offset`——恰好与这里的手腕一致。ROS 侧要不要也换成合并模型,留待决定。
- `head_yaw_Link` 的网格在两份导出里前后颠倒(脖子支架一个伸向前、一个伸向后),这里用的是 26.8.16_3 的;
  头部其余部分与相机两份一致。真机若与 7.24 一致需再换。
