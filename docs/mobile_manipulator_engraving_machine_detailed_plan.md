# 移动机械臂雕刻机自动上下料系统详细方案

> **项目目标**：使用一台带移动底盘的机械臂，从原料区抓取物料，运输至雕刻机完成自动上料；加工完成后自动下料，并将成品送到指定区域。
> **已落地技术路线**（2026-07 仿真全链路验证）：ROS 2 Jazzy + slam_toolbox 建图 + Nav2（NavFn 全局规划 + MPPI 控制）+ AprilTag（apriltag_ros，真图像解码）+ TF2 + MoveIt 2（moveit_py 嵌入式）+ 机器桥（MachineIO 抽象，仿真后端 / 真机 Modbus 后端）+ YAML 数据驱动状态机。仿真世界为 Isaac Sim（传感器级：真渲染、真雷达），控制栈与真机同构。
> **总体原则**：先用确定性方法把系统做稳定，再把 VLA 放到随机物料识别、技能选择等真正需要泛化的部分。VLA 本期未接入（按约定排除）。
>
> **文档状态**：本文档已按实际实施修订——"推荐/待选"处改为实际采用的做法，并补充实测数据与实施笔记。代码见 `Rviz/src/franzi_*` 五个包与 `IssacSim/`，操作手册见 `Rviz/src/franzi_skills/README.md`。

---

## 目录

1. 项目概述
2. 需求与边界
3. 总体流程
4. 系统架构
5. 硬件方案
6. 坐标系与 TF
7. 相机、AprilTag 与手眼标定
8. 底盘导航与精停靠
9. 机械臂抓取、上料和下料
10. MoveIt 2 使用方案
11. 夹具与机械容错
12. 雕刻机通信与安全联锁
13. 状态机设计
14. 异常恢复
15. ROS 2 软件架构
16. 关键算法
17. 数据记录与监控
18. 开发实施阶段
19. 测试与验收
20. 风险清单
21. VLA 接入位置
22. 待确认参数

---

# 1. 项目概述

本项目属于典型的 **移动操作机器人（Mobile Manipulator）自动上下料系统**。机器人需要在原料区、雕刻机和成品区之间移动，并完成抓取、运输、上料、加工等待、下料和成品放置。

完整任务链路：

```text
原料区
  ↓ 抓取原料
底盘运输
  ↓
雕刻机工位
  ↓ 精确停靠
机械臂上料
  ↓
雕刻机加工
  ↓
机械臂下料
  ↓
底盘运输
  ↓
成品区放置
```

整个项目涉及：

- 移动底盘导航和定位
- 工位精确停靠
- 相机内参和手眼标定
- 坐标系与齐次变换
- 机械臂 IK、避障和轨迹执行
- 抓取与放置
- 雕刻机 PLC/IO 通信
- 状态机与异常恢复
- 工业安全联锁

---

# 2. 需求与边界

## 2.1 核心功能

系统至少应实现：

1. 接收一条加工任务。
2. 导航至原料区。
3. 确定目标物料位置。
4. 机械臂抓取并确认成功。
5. 机械臂回到运输安全位姿。
6. 导航至雕刻机附近。
7. 使用 AprilTag 或机械结构完成精确停靠。
8. 检查雕刻机是否允许上料。
9. 将物料放入夹具。
10. 确认物料到位、夹具闭合。
11. 机械臂退出危险区域。
12. 启动加工。
13. 等待加工完成。
14. 确认主轴停止、门打开、夹具松开。
15. 取出成品。
16. 导航到成品区。
17. 放置成品并更新格位状态。
18. 返回待命点或继续下一任务。

## 2.2 第一版建议约束

MVP 阶段建议：

- 物料类型固定或数量较少。
- 物料不堆叠，统一平放。
- 原料和成品工位固定。
- 雕刻机夹具结构固定。
- 先只接一台雕刻机。
- 采用顶部抓取。
- 运行区域地图固定。
- 关键安全环节不用 VLA。
- 精密插入动作使用确定性轨迹和机械导向。

## 2.3 验收指标与仿真实测

| 指标                                |        目标 |                        仿真实测（2026-07，Isaac 真视觉链路） |
| ----------------------------------- | ----------: | -----------------------------------------------------------: |
| 完整任务连续成功率                  |      ≥ 95% |             单周期已稳定跑通（IDLE→DONE），连续统计留待真机 |
| 抓取成功率                          |      ≥ 98% |                     全周期内抓取/上料/下料/放置各 1 次均成功 |
| 底盘粗导航位置误差                  |    ≤ 50 mm | Nav2 到位 ~250 mm（goal tolerance），随后里程计补位至 ~30 mm |
| 精停靠位置误差                      | ≤ 5～10 mm |                          **0.5～6 mm**（四次停靠实测） |
| 精停靠偏航误差                      | ≤ 0.5～1° |                                          **≤ 0.28°** |
| 工件观测误差（tag→抓取点 vs 勘测） |          — |                     **4～7 mm**（apriltag_ros 真解码） |
| 通信错误导致误启动                  |           0 |                                0（互锁双侧执行，超时即失败） |

安全事故类指标属真机阶段;仿真为运动学级（无接触力），插入容差由夹具清隙(±12 mm)兜底。

---

# 3. 总体流程

实际业务状态（`franzi_task_manager/config/states.yaml`，数据驱动，加载即校验）：

```text
IDLE → SYSTEM_CHECK
  → GO_RAW_STATION → DOCK_RAW_STATION → DETECT_MATERIAL
  → PICK_MATERIAL → VERIFY_PICK → MOVE_TRANSPORT_POSE
  → GO_MACHINE → DOCK_MACHINE → CHECK_MACHINE_READY
  → OPEN_DOOR → OPEN_CLAMP → LOAD_MACHINE → CLOSE_CLAMP
  → EXIT_MACHINE → CLOSE_DOOR → START_MACHINING
  → GO_HOME_WAIT → WAIT_MACHINING            # 加工期间回待命点等
  → GO_MACHINE_UNLOAD → DOCK_MACHINE_UNLOAD
  → OPEN_DOOR_UNLOAD → OPEN_CLAMP_UNLOAD → UNLOAD_MACHINE
  → VERIFY_UNLOAD → EXIT_MACHINE_UNLOAD
  → GO_OUTPUT → DOCK_OUTPUT → PLACE_PRODUCT
  → RETURN_HOME → GO_HOME_END → DONE
任一步失败 → 按表重试 → RECOVERY（回安全姿态并停下）→ FAULT（等人工）
```

相对初版的调整：门/夹具开合拆成独立的 service 状态（每步等确认信号）;
加工等待期机器人**返回 home 待命点**，加工完再回机器取件;下料后显式收臂
（EXIT_MACHINE_UNLOAD）再开车。

实际技能层为 8 个 ROS 2 Action（`franzi_engraving_interfaces/action`，
由 `franzi_skills` 的 skill_server 统一提供，同一时刻只执行一个）：

```text
navigate_to_station   # 粗导航（Nav2 NavigateToPose；RViz 后端为运动学直驱）
precise_dock          # AprilTag 视觉闭环停靠
detect_material       # tag 定位工件，回报抓取位姿与勘测偏差
pick_material / load_machine / unload_machine / place_product
move_to_safe_pose     # transport / machine_safe 姿态
```

“启动加工 / 等加工完成”不是技能而是机器桥的 service + 状态表的 wait 态；
recover 也不是独立技能，而是状态表里的 RECOVERY 路由（move_safe + 停机）。
每个技能支持反馈、取消、超时和错误码，任务层只认错误码路由。

---

# 4. 系统架构

## 4.1 任务层

负责：

- 任务接收
- 流程编排
- 状态切换
- 超时
- 重试
- 异常恢复
- 人工接管

实现方式：

- 有限状态机
- BehaviorTree.CPP
- ROS 2 Lifecycle Node
- PLC 作为主状态机，ROS 负责技能执行

## 4.2 技能层

提供：

- 导航技能
- 精停靠技能
- 抓取技能
- 上料技能
- 下料技能
- 放置技能
- 安全撤离技能
- 复位技能

## 4.3 规划与控制层

- Nav2：底盘粗导航
- 精停靠控制器：根据 Tag 位姿闭环控制底盘
- MoveIt 2：机械臂 IK、碰撞检测和路径规划
- Cartesian Path/MoveIt Servo：最终直线接近和退出
- ros2_control 或厂商 SDK：执行轨迹
- PLC：设备联锁

## 4.4 感知与标定层

- RGB/RGB-D 相机
- AprilTag 检测
- 物料检测
- PnP 位姿估计
- 相机内参
- 手眼标定
- 工位外参
- TF2

## 4.5 安全与设备层

- 底盘
- 机械臂
- 夹爪
- 雕刻机
- PLC
- 自动门
- 自动夹具
- 安全雷达
- 急停
- 安全继电器或安全 PLC

---

# 5. 硬件方案（实际平台：Franzi 人形移动机器人）

## 5.1 移动底盘（实际：三轮独立转向全向底盘）

Franzi 底盘为三轮独立转向（holonomic），可直接控制 \(v_x,v_y,\omega_z\)——
与初版分析一致，全向在工位精调上占优。仿真中底盘由运动学驱动器实现
（`franzi_pick_place/base.py`）：吃 `/cmd_vel`、发 `/odom` 与
`odom→moveit_root` TF，接口与真机驱动完全一致；精停靠的离散小步修正在
真机上换成同一决策器驱动的连续速度伺服。

真机仍需核对：编码器/IMU、急停、防撞条、电量、刹停晃动。

## 5.2 机械臂（实际：双臂人形，左臂作业）

- 左臂 7 DoF 执行全部抓放（冗余自由度由 IK 分支选择管理，见 §10.5）；
- 躯干 3 关节（waist/thigh/calf）提供工作姿态，头部 2 关节承载观测；
- 作业半径实测覆盖：停靠位到夹具中心 0.44 m（`dock.offset`，经
  reach_map 工具验证在包络内）；
- 已核对：伸入机器不接近奇异点（同分支 IK + 直线插补验证）、运输姿态
  为持料 carry 位（臂举于胸前，见 §9.1）。

## 5.3 夹爪（实际：双指平行爪）

两指对开，零位开度 95 mm、单指行程 47.5 mm；抓取开度 85 mm，闭合到
工件宽度 −2 mm（grasp_squeeze）。经 FollowJointTrajectory 直接驱动
（闭合本来就是有意接触，不走规划器）。真机补充：到位/夹持力反馈用于
VERIFY_PICK（仿真中以关节到位代替）。

## 5.4 相机（实际：头/胸 D435 + 双腕 D405）

- **头部 D435**（1280×720）：承担全部站位 tag 停靠与工件定位。注意实际
  机构头部俯仰限位 ±30°（已到底），加装配俯角后视轴距竖直 40°——该视角
  下 0.85 m 处的 80 mm tag 约 80 px，apriltag decimate 必须设 1.0；
- **胸部 D435、双腕 D405**：本期用于录制与预留（腕部近距二次修正是
  后续增强的正路，框架已就位）；
- 深度未参与定位：工件平放固定面，RGB+tag 平面假设即可（与初版判断
  一致）；D405 深度流已发布备用。

实施笔记：URDF 相机 link 为"z 出镜头"的类光学朝向，但其 x 轴非图像行
方向——光学帧 = link ∘ Rz(−90°)，由 launch 发布静态 TF，Isaac 侧同构
旋转并在启动时打印同一变换做交叉核对。

## 5.5 雕刻机侧信号

建议具备：

- MachineReady
- DoorOpen
- DoorClosed
- ClampOpen
- ClampClosed
- MaterialDetected
- SpindleStopped
- Machining
- Finished
- Alarm
- EmergencyStop

---

# 6. 坐标系与 TF

## 6.1 推荐 TF 树

```text
map
└── odom
    └── base_link
        ├── laser_link
        ├── front_camera_link
        │   └── front_camera_optical_frame
        └── arm_base
            └── arm_link_1
                └── ...
                    └── tool0
                        └── tcp
                            └── wrist_camera_link
                                └── wrist_camera_optical_frame
```

工位坐标：

```text
map
├── raw_station
│   ├── raw_tag_board
│   ├── raw_tray
│   └── raw_pick_pose
├── engraving_station
│   ├── machine_tag_board
│   ├── fixture
│   ├── preload_pose
│   ├── load_pose
│   ├── unload_pose
│   └── safe_exit_pose
└── output_station
    ├── output_tag_board
    ├── output_tray
    └── output_place_pose
```

## 6.2 齐次变换

\[
{}^A_BT=
egin{bmatrix}
{}^A_BR & {}^A\mathbf t_B
0&1
\end{bmatrix}
\]

点的变换：

\[
{}^A	ilde
=========

{}^A_BT\,{}^B	ilde{\mathbf p}
\]

连续变换：

\[
{}^A_DT
=======

{}^A_BT\,{}^B_CT\,{}^C_DT
\]

判断顺序：

> 右边是起点，左边是终点；中间坐标系必须首尾相接。

## 6.3 抓取位姿计算

相机检测：

\[
{}^{camera}T_{object}
\]

手眼标定：

\[
{}^{arm\_base}T_{camera}
\]

物体到抓取位姿偏移：

\[
{}^{object}T_{grasp}
\]

最终：

\[
{}^T_
=====

{}^{arm\_base}T_{camera}
{}^{camera}T_{object}
{}^{object}T_{grasp}
\]

## 6.4 雕刻机夹具位姿

前视相机检测：

\[
{}^{camera}T_{tag}
\]

已知相机到底盘：

\[
{}^{base}T_{camera}
\]

已知 Tag 到夹具：

\[
{}^{tag}T_{fixture}
\]

则：

\[
{}^T_
=====

{}^{base}T_{camera}
{}^{camera}T_{tag}
{}^{tag}T_{fixture}
\]

机械臂坐标系下：

\[
{}^T_
=====

({}^{base}T_{arm\_base})^{-1}
{}^{base}T_{fixture}
\]

---

# 7. 相机、AprilTag 与手眼标定

## 7.1 实际检测方案

- 每站一枚 **tag36h11**（feeder=0、machine=1、outfeed=2），80 mm，平贴
  台面；检测器为 **apriltag_ros**（真图像解码，Isaac 渲染流）；
- 检测发布为 TF 帧 `tag_N`——与 RViz 几何级仿真的 mock 检测器同名同帧，
  上层（TagObserver/技能）对二者零感知，这就是"换检测器不换代码"的接缝。

**实施笔记（真机同样会踩）**：umich apriltag（apriltag_ros 所用）与
OpenCV ArUco 对 36h11 图案帧的定义**相差绕法线半圈**。勘测若按一种约定
做，tag 板必须按同一约定的方向安装，否则 tag→工件偏移方向整体反转
（症状：观测出的工件在 tag 另一侧、停靠目标 yaw≈180°）。本项目勘测按
apriltag_ros 约定，Isaac 场景中贴图旋转 180° 安装。

## 7.2 Tag 尺寸与解码余量（实测）

工作距离 0.85 m、头相机 fx≈931 px 下 80 mm tag 成像约 80 px，
`decision_margin≈170`（阈值一般取 30），余量充足；但 **decimate 必须
设 1.0**（默认 2.0 会把有效分辨率砍半到解不出码）。粗导航 0.25 m 容差
处 tag 仅 ~40 px 不可解——因此停靠技能先按里程计补位到示教点再开始
视觉闭环（见 §8）。

多 tag board（4 枚）作为真机抗遮挡/降抖动的增强保留，本期单 tag 已满足
精度（观测误差 4–7 mm）。打印后实测尺寸的要求不变——尺寸差 1%，
距离就差 1%。

雕刻机站的 tag 有独立勘测偏移（`tag.machine_to_part_xy`）：夹具位于
主轴正下方的工作台深处，tag 需贴在台板外侧可视处、横向偏移压小
（停靠位半视场 ~34°，贴到台角会整体出画）。

## 7.5 安装要求

- 固定在刚性结构
- 避免金属强反光
- 避免粉尘直喷
- 不贴在会抖动的薄板上
- 定期清洁
- 尽量保证底盘多个角度都能看到
- Board 与夹具的外参要单独标定

## 7.6 相机内参标定

需要求：

\[
K=
egin{bmatrix}
f_x&0&c_x
0&f_y&c_y
0&0&1
\end{bmatrix}
\]

以及：

```text
k1, k2, p1, p2, k3
```

建议：

- 20～40 张棋盘格图像
- 不同角度和距离
- 覆盖画面四角
- 重投影误差尽量小于 1 px

## 7.7 手眼标定

### 前视相机固定在底盘

求：

\[
{}^{base}T_{camera}
\]

### 腕部相机装在末端

求：

\[
{}^{eef}T_{camera}
\]

手眼标定解决的是：

> 相机装在机器人上的位置和方向。

它不等于运行时工位定位。

## 7.8 为什么标定完仍然需要 AprilTag

手眼标定得到固定关系：

\[
{}^{base}T_{camera}
\]

底盘每次停靠相对雕刻机的位置却不同。AprilTag 每次运行提供：

\[
{}^{camera}T_{machine}
\]

因此：

\[
{}^T_
=====

{}^{base}T_{camera}
{}^{camera}T_{machine}
\]

所以：

- 手眼标定：通常安装后做一次
- AprilTag：每次到工位后检测

## 7.9 Tag 到夹具外参

需要确定：

\[
{}^{tag}T_{fixture}
\]

可通过 CAD、精密测量、三坐标、机械臂 TCP 触碰特征点或专用标定治具。
这项误差会直接进入最终放料误差。

**实施笔记**：手眼标定结果在本实现中是**独立校正 TF 帧**
（`<camera>_calibrated`，叠在 URDF 名义外参之上），标定错多少、抓取
就偏多少并会如实显示在每次检测的"与勘测偏差"日志里（>100 mm 触发
SURVEY_MISMATCH 拒抓）——这条 deviation 数字就是真机标定的质检工具。
仿真中手眼校正为零位；真机标定后填同一组参数即可。

---

# 8. 底盘导航与精停靠

## 8.1 实际为三阶段方案

```text
Nav2 粗导航(地图 + NavFn 规划 + MPPI 控制,到位容差 ~0.25 m)
  ↓
里程计补位:低速直驱到示教停靠点(残差 ~30 mm)
  ↓
AprilTag 视觉闭环(压到毫米级) → 稳定帧确认 → 机械臂开始工作
```

中间的"里程计补位"是实施中加出来的一环:Nav2 的 0.25 m 到位容差下,
80 mm tag 只有 ~40 px 解不出码;先按里程计走完最后一段,tag 就回到设计
解码距离(0.85 m,~80 px)。真机上这一段就是低速直驱到示教位。

## 8.2 建图与粗导航（实际配置）

- **建图**:slam_toolbox（online_async）+ 2D 雷达 `/scan`（SICK TIM781
  仿真配置）。巡线为**实地全程大环线**——从墙角待命点出发,沿南墙、
  工位走廊、北墙、西侧走廊绕回墙角(约 55 m),机器人将来会开到的每片
  区域都获得近距多角度覆盖,不依赖远距扫描回波;地图存
  `franzi_skills/maps/cell`;
- **定位**:仿真中为静态恒等 `map→odom`——运动学 odom 无漂移即真值,
  AMCL 的扫描时序敏感性只添脆性;**真机换回 AMCL / slam_toolbox
  localization**,地图、参数、示教位姿全部不变;
- **Nav2 组成**:planner_server(NavFn) + controller_server(MPPI) +
  behavior_server + bt_navigator 四件套,controller 直出 `/cmd_vel`;
  velocity_smoother 与 collision_monitor 留待真机 bringup(本机
  smoother configure 存在环境性缺陷,monitor 依赖的扫描时序正是本
  阶段刻意解耦的);costmap 用纯静态层(场景无动态障碍,真机加回
  obstacle 层);
- **导航目标**:示教停靠位记录在 odom 系,发目标时经当前 `map←odom`
  变换换算——定位怎么漂,车都物理到达示教点(后续 tag 闭环和机械臂
  要的是这个);
- 粗导航只需进入 tag 可见/臂可达/闭环可控范围,与初版一致。

## 8.3 位姿误差

\[
e_x=x_d-x
\]

\[
e_y=y_d-y
\]

\[
e_	heta=
\operatorname{atan2}
\left(
\sin(	heta_d-	heta),
\cos(	heta_d-	heta)

ight)
\]

yaw 不能直接普通相减，否则在 \(+\pi/-\pi\) 附近会出问题。

## 8.4 全向底盘

\[
v_x=K_xe_x,\quad
v_y=K_ye_y,\quad
\omega=K_	heta e_	heta
\]

必须增加：

- 速度限幅
- 最小速度
- 加速度限制
- 到位死区
- 多帧稳定判断

## 8.5 差速底盘

推荐分阶段：

```text
先对准 yaw
  ↓
前后移动
  ↓
小弧线修正横向误差
  ↓
再次修正 yaw
```

## 8.6 精停靠状态机

```text
SEARCH_TAG
  ↓
TAG_VISIBLE
  ↓
ALIGN_YAW
  ↓
ALIGN_POSITION
  ↓
FINE_ADJUST
  ↓
SETTLE
  ↓
DOCKED
```

## 8.7 到位条件（实际值）

```text
|ex| < 8 mm, |ey| < 8 mm, |eθ| < 0.8°
连续满足 N 帧:仿真 N=3(每帧含 1 s 静止观测,节拍慢),真机建议回到 15
实测停靠残差:0.5～6 mm / ≤0.28°
```

补充实施要点:每步修正限幅(单步 ≤0.15 m / 12°),死区 2 mm;修正期间
**停稳后才观测**——运动中读 tag 会因图像与 TF 时间戳错位产生厘米级
幻影误差(实测复现过),这也是节拍呈"走-停-看"的原因。决策逻辑在
`franzi_skills/dock_controller.py`,纯函数、全分支单测覆盖;执行器注入
(仿真=小步直驱,真机=连续 cmd_vel 伺服),换执行器不换决策。

## 8.8 位姿滤波

可用：

- 中值滤波
- EMA
- Kalman Filter
- 重投影误差过滤
- 多 Tag 融合
- 跳变拒绝

简单 EMA：

\[
x_f=lpha x+(1-lpha)x_{f,prev}
\]

建议：

```text
alpha = 0.2～0.5
```

## 8.9 底盘稳定检测

机械臂动作前确认：

- `cmd_vel = 0`
- 里程计速度接近 0
- IMU 角速度接近 0
- 底盘已刹车
- 可选支撑脚已落下
- 稳定持续一定时间

---

# 9. 机械臂抓取、上料和下料

## 9.1 通用动作结构

```text
安全位姿
  ↓
预接近位姿
  ↓
笛卡尔直线接近
  ↓
抓取/放置
  ↓
笛卡尔直线退出
  ↓
安全位姿
```

## 9.2 原料抓取

1. 底盘停稳。
2. 机械臂到原料区安全位。
3. 相机检测候选物料。
4. 用户点选或系统自动选择。
5. 计算抓取位姿。
6. MoveIt 到预抓取位。
7. 笛卡尔直线下降。
8. 夹爪闭合或吸盘建立真空。
9. 检查抓取反馈。
10. 直线上升。
11. 将物料 attach 到夹爪。
12. 机械臂回运输位。

## 9.3 雕刻机上料

1. 检查 MachineReady。
2. 检查 SpindleStopped。
3. 请求开门。
4. 确认 DoorOpen。
5. 请求夹具打开。
6. 确认 ClampOpen。
7. 检查 Docked。
8. 到机外安全位。
9. MoveIt 到预上料位。
10. 腕部视觉检查夹具。
11. 低速直线进入。
12. 放下物料。
13. 检查 MaterialDetected。
14. 请求夹具闭合。
15. 确认 ClampClosed。
16. 直线退出。
17. 到机外安全位。
18. 置位 RobotClear。
19. 关门。
20. 启动加工。

## 9.4 雕刻机下料

1. 等待 Finished。
2. 确认 SpindleStopped。
3. 请求开门。
4. 确认 DoorOpen。
5. 请求夹具松开。
6. 确认 ClampOpen。
7. 到预抓取位。
8. 直线进入。
9. 夹持成品。
10. 检查抓取成功。
11. 直线退出。
12. 回运输位。
13. 置位 RobotClear。

## 9.5 成品放置

固定托盘方案：

- 每个格位定义固定坐标
- 维护占用状态
- 选择下一个空位
- 放置后更新状态

---

# 10. MoveIt 2 使用方案

MoveIt 2 主要负责：

- IK
- 碰撞检测
- 关节限位
- 运动规划
- 时间参数化
- 轨迹执行
- Attached Collision Object

## 10.1 大范围动作

使用 MoveIt：

```text
Home → Pre-Grasp
Transport → Machine Safe Pose
Machine Safe Pose → Pre-Load
Machine Safe Pose → Transport
```

## 10.2 最终接近

最后几厘米建议使用：

- Cartesian Path
- MoveIt Servo
- 自定义直线轨迹

普通关节规划不保证末端走直线，不适合直接插入夹具。

## 10.3 Planning Scene

加入：

- 雕刻机外壳
- 门框
- 工作台
- 夹具
- 托盘
- 底盘
- 相机支架
- 周围固定障碍物

第一版可用 Box/Cylinder 近似，碰撞模型建议略微放大 10～30 mm。

## 10.4 Attached Object

抓住物料后必须 attach，否则 MoveIt 不知道长板或工件会随夹爪移动。

```text
抓取成功
  ↓ attach
运输/上料
  ↓
释放
  ↓ detach
```

## 10.5 IK 连续性

- 使用当前关节角作为 seed
- 优先选择离当前构型最近的解
- 避免手腕翻转
- 避免奇异点
- 对 continuous joint 选择最近等价角

## 10.6 速度建议

自由空间：

```text
speed_scale = 0.2～0.5
```

机器内部：

```text
speed_scale = 0.05～0.15
```

最终参数需要根据机械臂、负载和安全规范确认。

---

# 11. 夹具与机械容错

夹具设计往往比继续提高视觉精度更有效。

推荐：

- 定位销
- V 型槽
- 三点定位
- 倒角
- 漏斗形入口
- 限位挡块
- 防反装
- 自动夹紧
- 到位传感器

建议将最后误差留给机械结构消化：

| 类型 |  建议容差 |
| ---- | --------: |
| X/Y  | ±3～5 mm |
| Z    | ±2～3 mm |
| Yaw  |  ±2～3° |

可增加：

- 浮动法兰
- 弹性指尖
- 被动柔顺
- RCC 远心柔顺机构
- 力传感器
- 阻抗控制

对于插入式动作，机械导向和柔顺通常非常值。

---

# 12. 雕刻机通信与安全联锁

## 12.1 通信方式（实际：机器桥 + MachineIO 抽象）

实现为独立节点 `franzi_machine_bridge`：对 ROS 侧提供
`machine/status` 话题与 `machine/set_door` / `set_clamp` /
`start_machining` / `reset_fault` 服务；对机器侧只经一个
**MachineIO 抽象**（read 信号 / write 信号 / command 动作）通信。

- 仿真后端 `SimulatedMachine`：门 2.5 s、夹具 1.2 s、主轴停转 2 s、
  加工 8 s——动作有时长、双限位信号在运动中同时为假，逼出调用方
  所有该有的等待；
- 真机后端＝实现同一 MachineIO 的 Modbus TCP / 数字 IO 类（优先级
  与初版一致），**桥、互锁、任务层零改动**；
- 铁律：桥绝不虚构信号——机器报不了的字段保持 False，宁可互锁拒绝。
- 仿真专用接缝：`machine/sim/material_present` 话题由技能层在放取
  工件时发布，模拟机器自己的到位传感器；真机上无人发布此话题，
  传感器经 MachineIO 上报。

## 12.2 雕刻机发给机器人

```text
MachineReady
DoorOpen
DoorClosed
ClampOpen
ClampClosed
MaterialDetected
Machining
Finished
SpindleStopped
Alarm
EmergencyStop
```

## 12.3 机器人发给雕刻机

```text
LoadRequest
UnloadRequest
StartRequest
RobotClear
RobotBusy
RobotFault
ResetRequest
```

## 12.4 启动加工条件

必须同时满足：

```text
MaterialDetected = true
ClampClosed = true
DoorClosed = true
RobotClear = true
Alarm = false
EmergencyStop = false
```

## 12.5 机械臂进入条件

必须同时满足：

```text
SpindleStopped = true
DoorOpen = true
ClampOpen = true
Alarm = false
EmergencyStop = false
```

## 12.6 超时（已按此实现）

```text
等待开门/关门：10 s      等待夹具开/合：5 s
等待物料到位：3 s        等待加工完成：工艺 8 s + 余量(表值 120 s)
```

桥内每个动作**等确认信号，超时即失败**（发出请求不算成功）；任务层
另有一层状态超时。互锁在**两侧独立执行**：桥拒绝在不安全条件下动门/
启动（§12.4/12.5 的条件即 `interlocks.py` 的纯函数，单测覆盖），技能层
在机械臂跨入机器包络前自查同一组条件——互不信任对方查过。
`robot_clear` 由任务层**独占发布**（全量消息、增量语义），由状态表的
signals_on_entry/on_success 编排升降；FAULT 态刻意不置位——故障的臂
可能停在机器里。

## 12.7 安全硬件

软件状态机不能替代：

- 急停
- 安全继电器
- 安全 PLC
- 安全门
- 安全雷达
- 限速
- 机械限位
- 主轴硬件停止确认

---

# 13. 状态机设计（实际实现）

状态机是**一份 YAML 数据表**（`franzi_task_manager/config/states.yaml`，
全部状态见 §3），不是代码：任务节点只做"查入口条件 → 派动作 → 计超时
→ 数重试 → 按表路由失败"，对臂、tag、夹具零知识。表在加载时自校验
（无出口的状态、指向不存在状态的转移都拒绝加载）。动作分四类：
`skill`（调技能 Action）/ `service`（调机器桥服务）/ `wait`（等信号）/
`none`。

实际状态条目字段与示例（LOAD_MACHINE 现行定义）：

```yaml
LOAD_MACHINE:
  action: {kind: skill, skill: load_machine, station: machine}
  # 入口互锁写在表里让周期在动臂前就停下；技能内部还会自查一遍
  entry_conditions: [spindle_stopped, door_open, clamp_open,
                     not alarm, not emergency_stop]
  # 臂即将跨入机器包络：robot_clear 先拉低
  signals_on_entry: {robot_clear: false}
  success_conditions: [material_detected]   # 机器的传感器说了算
  timeout: 40.0
  max_retry: 1
  error_code: LOAD_FAILED
  failure_transition: RECOVERY
  next: CLOSE_CLAMP
```

相对初版的语义决策：

- 一个状态**从不默认成功**——动作报成功且 success_conditions 成立才算；
  超时即失败，哪怕等的东西事后到了；
- `signals_on_entry / signals_on_success` 把 robot_clear 的升降编进表
  （EXIT_MACHINE 成功才升高），任务层是 robot/signals 唯一写者；
- RECOVERY 不自动续任务：回安全姿态并停下，续不续由人决定——在真机
  失败模式摸清之前，这是正确的保守；
- 导航类状态 max_retry=2、超时 90 s（真 Nav2 有启动竞态与负载抖动，
  技能层内部对"目标被拒/立即失败"另有 4 次快速重试吸收）。

---

# 14. 异常恢复（已实现部分按实际记）

## 14.1 Tag 丢失（已实现）

停靠闭环内：连续 2 次观测失败先原地等待，再后退 80 mm 重找，最多 3 次
后报 `TAG_LOST`；观测总迭代设发散上限（防转圈）。停靠开始前的"里程计
补位到示教点"本身就消掉了大部分"太远看不见"的丢失。任务层对
DOCK_* 状态整体重试 2 次，仍失败走 RECOVERY。

## 14.2 抓取失败（部分实现）

仿真为运动学抓取，实现了：夹爪到位即 `holding` 校验（VERIFY_PICK）、
PICK 状态任务级重试 1 次（重试内会重新观测）。真机补充：夹持力/行程
反馈、负载校验、异常物料标记。

## 14.3 物料未到位（已实现）

LOAD 的成功条件就是机器的 `material_detected` 传感器（3 s 超时），
不满足即失败重试；夹具闭合的成功条件同时要求 `clamp_closed ∧ material_detected`——没到位就永远走不到 START_MACHINING。

## 14.4 IK/规划失败（已实现，含实施补充）

- 当前关节角作 seed、失败扰动重试（25 次）；预抓取与接触位强制同一
  IK 分支且**先验证直线可达**才接受（防手腕翻转与"下降绕大圈"）；
- 直线段失败自动回退自由空间规划；
- **接触窗口豁免**（实施新增）：抓/放期间，工件×夹指×夹具墙×台面的
  接触对临时放行，且豁免保持到臂撤回安全姿态才关闭——否则"手指
  尚在已放工件两侧"的真实状态会被判为碰撞起点，任何规划都无法出发;
- **放置后撤离失败降级**（实施新增）：工件已放好时 retreat 失败只告警,
  收臂交给下一状态的安全姿态动作——避免重试逻辑对着空夹爪重放。

## 14.5 导航失败（实施新增）

技能内对"目标被拒/立即失败"快速重试 4 次（吸收 Nav2 启动竞态与
瞬时负载抖动），任务级再重试 2 次；仍失败 RECOVERY。

## 14.5 雕刻机报警

- 禁止进入机器
- 停止底盘
- 机械臂回安全位
- 记录报警码
- 等待人工复位

## 14.6 网络或节点异常

设置 Watchdog：

```text
心跳丢失
  ↓
底盘速度置零
  ↓
机械臂停止/保持
  ↓
撤销 PLC 启动许可
  ↓
进入 Fault
```

---

# 15. ROS 2 软件架构（实际）

## 15.1 包与节点

```text
franzi_engraving_interfaces  msg/srv/action 定义(全部接口的单一来源)
franzi_task_manager          task_manager:YAML 状态机 + 技能/服务客户端
franzi_skills                skill_server:8 个技能 Action(独占臂/底盘/场景,
                             内嵌 MoveItPy、底盘驱动、tag 管线、格位簿)
                             camera_recorder / mapping_drive 工具节点
franzi_machine_bridge        machine_bridge:互锁 + 超时 + MachineIO 后端
franzi_pick_place            构件库(底盘、MoveIt 封装、夹爪、场景、tag、参数表)
                             + 原 RViz 几何级演示
franzi_moveit_config / franzi_description   MoveIt 配置与 URDF
外部:move_group、apriltag_ros、Nav2 四件套、map_server、slam_toolbox、
      robot_state_publisher、Isaac(传感器级仿真,镜像执行)
```

单一 launch 入口:`franzi_skills/launch/tending_cell.launch.py`,开关
`backend:=rviz|isaac`(mock 检测 / 真检测)与 `nav:=true|false`。

## 15.2 Topics(实际命名)

```text
/head_d435/color/image_raw|camera_info  /head_d435/depth/image_rect_raw
/body_d435/... /left_wrist_d405/... /right_wrist_d405/...   # RealSense 布局
/scan  /mid360/points        # 2D / 3D 雷达
/cmd_vel  /odom  /joint_states  /base/pose
/machine/status  /robot/signals  /task/state
/machine/sim/material_present  /workpiece/pose   # 仿真专用接缝
TF: map→odom(定位) odom→moveit_root(底盘) <camera>_optical tag_N
```

## 15.3 Actions(8 个技能 + 导航)

```text
navigate_to_station  precise_dock  detect_material  pick_material
load_machine  unload_machine  place_product  move_to_safe_pose
(Nav2 的 navigate_to_pose 由 navigate_to_station 内部调用)
```

## 15.4 Services

```text
machine/set_door{close}  machine/set_clamp{close}   # 开合合一,等确认信号
machine/start_machining(带 blocked_by 回执)  machine/reset_fault
task/clear
```

---

# 16. 关键算法

> 实现对照：16.1/16.3 即 `franzi_skills/dock_controller.py`（纯函数，
> 全分支 pytest 覆盖，另加了限步、死区、丢失后退与发散上限）；
> 16.2 即 tag 管线（`franzi_pick_place/tags.py` + 技能 observe_part）；
> 16.4/16.5 即 `franzi_machine_bridge/interlocks.py`（原样实现，单测覆盖）。

## 16.1 周期角误差

```python
import numpy as np

def shortest_angle_diff(target: float, current: float) -> float:
    return np.arctan2(
        np.sin(target - current),
        np.cos(target - current),
    )
```

## 16.2 抓取位姿变换

```python
T_arm_camera = lookup_tf("arm_base", "camera")
T_camera_object = detect_object_pose()
T_object_grasp = load_grasp_offset()

T_arm_grasp = (
    T_arm_camera
    @ T_camera_object
    @ T_object_grasp
)
```

## 16.3 精停靠伪代码

```python
while running:
    detection = get_tag_board_pose()

    if detection is None:
        stop_base()
        if tag_lost_too_long():
            if not recover_tag():
                raise DockingError("Tag lost")
        continue

    pose = filter_pose(detection)
    ex, ey, eyaw = compute_error(pose, target_pose)

    if (
        abs(ex) < x_tol
        and abs(ey) < y_tol
        and abs(eyaw) < yaw_tol
    ):
        stop_base()
        stable_count += 1

        if stable_count >= stable_frames:
            return DOCKED
    else:
        stable_count = 0

        vx = clip(kx * ex, -vx_max, vx_max)
        vy = clip(ky * ey, -vy_max, vy_max)
        wz = clip(kyaw * eyaw, -wz_max, wz_max)

        send_velocity(vx, vy, wz)
```

## 16.4 安全进入判断

```python
def can_enter_machine(status) -> bool:
    return all([
        status.machine_ready,
        status.spindle_stopped,
        status.door_open,
        status.clamp_open,
        not status.alarm,
        not status.emergency_stop,
    ])
```

## 16.5 启动加工判断

```python
def can_start_machine(status, robot_state) -> bool:
    return all([
        status.material_detected,
        status.clamp_closed,
        status.door_closed,
        robot_state.robot_clear,
        not status.alarm,
        not status.emergency_stop,
    ])
```

---

# 17. 数据记录与监控

每次任务建议记录：

- task_id
- material_id
- station_id
- 当前状态
- 状态切换时间
- 底盘位姿
- Tag 原始位姿
- Tag 过滤后位姿
- 停靠误差
- 机械臂 joint state
- EEF pose
- 夹爪状态
- PLC 信号
- 重试次数
- 错误码
- 关键截图
- 成功/失败标签

保存形式：

```text
rosbag2
+
JSON/CSV 事件日志
+
关键图像
```

监控界面建议显示：

- 当前任务
- 当前状态
- 底盘位置
- Docking 误差
- 机械臂状态
- 雕刻机状态
- 夹具状态
- 最近错误
- 节点心跳
- 电池电量

---

# 18. 开发实施阶段（实际执行路径回顾 + 真机待办）

仿真侧按下列顺序完成（与初版阶段划分对应关系标注在括号里）：

## 已完成（仿真）

1. **RViz 几何级闭环**（≈阶段 1+5 的运动学版）：MoveIt 抓放、场景、
   tag 管线（mock 检测）、底盘直驱，单脚本全流程；
2. **可复用化重构**（≈阶段 2+架构落地）：接口包、机器桥+互锁、YAML
   状态机、8 技能 Action 化；仿真机器时序（门 2.5 s/夹 1.2 s/加工 8 s）；
3. **Isaac 感知级**（≈阶段 4 的核心）：真渲染、apriltag_ros 真解码、
   光学帧标定链、雷达发布；观测误差实测 4–7 mm；
4. **建图与导航**（≈阶段 3+4）：slam_toolbox 建图、Nav2 四件套、
   里程计补位、tag 闭环停靠 0.5–6 mm；home 待命点进状态机；
5. **完整闭环 + 异常路径**（≈阶段 5+6 部分）：IDLE→DONE 全链路多次
   复现；tag 丢失恢复、导航重试、放置撤离降级、接触窗口豁免；
6. **演示交付**：上帝视角 / 机载四相机 / RViz 导航轨迹三部视频
   （`IssacSim/renders/`）。

## 真机待办（原阶段映射）

- 阶段 0 需求冻结:§22 表中"真机待定"各项;
- 硬件桥:franzi_hardware_bridge(MoveIt 控制器 ↔ franzi_sdk,见
  Rviz/NextStep.md)与 Modbus 版 MachineIO;
- 标定四件:相机内参、手眼(填校正帧参数)、tag 实测尺寸、tag→夹具
  外参;停靠位重新示教;
- 定位换 AMCL/slam localization,costmap 加回 obstacle 层,
  velocity_smoother/collision_monitor 启用;
- 阶段 6 剩余(机器报警注入、断链演练)与阶段 7(连续 100 次/8 h、
  节拍优化)——仿真已具备回归跑法(`cycles:=N`);
- 阶段 8 VLA 增强,接入点见 §21,本期按约定未做。

---

# 19. 测试与验收

> 现状：纯逻辑层 39 项单元测试（互锁/状态表/停靠闭环/格位簿）随构建
> 常跑；端到端以 `tending_cell.launch.py backend:=isaac nav:=true cycles:=N` 回归（DONE 即通过），单周期已多次复现。以下统计类测试
> （重复 20 次分布、连续 100 次、8 h）为真机/后续阶段项目。

## 19.1 AprilTag

测试：

- 不同距离
- 不同角度
- 不同光照
- 部分遮挡
- 运动模糊
- 粉尘
- 反光

统计：

- 检测率
- 位姿抖动
- 重投影误差
- 延迟
- 最大跳变

## 19.2 精停靠

从不同初始位置重复测试：

- 左偏
- 右偏
- 前后偏
- yaw 正负偏差

每种至少 20 次，统计：

- X/Y/yaw 平均误差
- 标准差
- 最大误差
- 95% 分位误差
- 平均停靠时间
- 失败率

## 19.3 抓取

固定底盘连续 100 次：

- 抓取成功率
- 滑落率
- 平均抓取时间
- 恢复成功率

## 19.4 上下料

固定底盘连续 100 次：

- 上料到位率
- 夹具闭合成功率
- 下料成功率
- 工件损伤率
- 碰撞次数

## 19.5 通信

模拟：

- 门不打开
- 夹具不闭合
- 主轴未停止
- 加工超时
- PLC 断线
- 报警
- 急停

验证系统不会错误进入下一状态。

## 19.6 完整系统

依次：

1. 单次完整循环
2. 连续 10 次
3. 连续 50 次
4. 连续 100 次
5. 连续运行 8 小时
6. 故障注入
7. 安全区域测试

---

# 20. 风险清单

| 风险           | 后果         | 对策                 |
| -------------- | ------------ | -------------------- |
| Tag 遮挡       | 无法精停靠   | 多 Tag Board         |
| Tag 尺寸不准   | 尺度误差     | 打印后实测           |
| 相机外参误差   | 系统性偏差   | 多姿态标定           |
| 底盘停车晃动   | 抓取漂移     | 等待稳定、锁止       |
| 地面打滑       | 重复性差     | 视觉闭环、机械定位   |
| IK 跳解        | 手腕翻转     | 当前关节作为 seed    |
| 欧拉角跳变     | 绕大圈       | 四元数和最短角差     |
| 碰撞模型缺失   | 撞机         | 完整 Planning Scene  |
| 未 attach 工件 | 工件撞机     | Attached Object      |
| 深度噪声       | Z 错误       | 平面约束、区域中值   |
| 门状态误判     | 严重安全风险 | PLC 硬联锁           |
| 主轴未停       | 严重安全风险 | 独立硬件确认         |
| 网络断线       | 状态不一致   | Watchdog             |
| 镜头污染       | 检测退化     | 防护罩和清洁         |
| 电量不足       | 中途停机     | 任务前检查           |
| 人员进入       | 人身风险     | 安全雷达、围栏、急停 |

---

# 21. VLA 接入位置

## 不建议交给 VLA

- 主轴安全
- 门和夹具联锁
- PLC 状态判断
- 急停
- 精停靠底层速度控制
- 最终插入安全限制

## 适合 VLA

- 随机物料抓取
- 多物料选择
- 自然语言任务
- 技能选择
- 异常视觉理解
- 非结构化放置
- 恢复策略建议

推荐架构：

```text
MES/用户指令
  ↓
高层 Planner / VLM
  ↓
选择确定性技能
  ↓
Nav2 / Dock / Pick / Load / Unload
  ↓
安全控制器
  ↓
PLC
```

---

# 22. 参数现状（仿真已定 / 真机待定）

## 机器人（仿真已定）

- 底盘：Franzi 三轮独立转向**全向**底盘（轮半径 0.0827 m）
- 机械臂：Franzi 双臂人形，作业臂＝左臂 7 DoF；躯干 3 关节、头 2 关节
  （头俯仰限位 ±30°，观测视角设计需迁就，见 §5.4）
- 夹爪：双指平行，零位开度 95 mm、行程 47.5 mm；力反馈：真机待定
- ROS 2：全栈 Jazzy；真机臂驱动经 franzi_sdk（桥待做，Rviz/NextStep.md）

## 物料（仿真设定）

- 尺寸 70×70×40 mm，平放不堆叠，顶抓，夹持位＝对边中部；
- 重量/材质/反光/夹持力：真机待定

## 雕刻机（仿真模型已定，真机全部待定）

- 门 2.5 s、夹具 1.2 s、主轴停转 2 s、加工 8 s（SimulatedMachine）；
- 槽位单边清隙 12 mm、槽墙高 15 mm；
- 信号集＝§12.2/12.3 全量；通信协议真机选型后写 Modbus 版 MachineIO

## 相机（仿真已定）

- 头 D435 1280×720（fx≈931）、胸 D435、双腕 D405 640×480；
- 头相机到工位 tag 观测距离 ~0.85 m；tag 80 mm 平贴台面

## 系统（仿真已定）

- Ubuntu 24.04 + ROS 2 Jazzy；仿真世界 Isaac Sim（IsaacLab 装配）；
- 单机部署；MES 对接、人机共线安全、目标节拍：真机阶段定

## 场地（仿真已定）

- 三站沿 y 排布：feeder (2.2,−4)、machine (2.2,0)、outfeed (2.2,4)；
- 停靠线 x=1.76（dock.offset 0.44/0.16）；
- **待命点＝墙角 (−7,−7)**，朝向车间中心——每次任务从这里出发，
  加工等待期与收尾也回到这里；
- 地图 slam_toolbox 建于同一场地（franzi_skills/maps/cell）

---

# 附录：实际运行参数（仿真档）

来源：`franzi_skills/config/skills.yaml`、`franzi_pick_place/config/task.yaml`、
`franzi_skills/config/nav2.yaml`。

```yaml
docking:                      # dock_controller 实际值
  position_tolerance: 0.008   # 8 mm
  yaw_tolerance_deg: 0.8
  stable_frames: 3            # 仿真档(每帧含 1s 静止观测);真机建议 15
  max_linear_step: 0.15       # 单步修正上限
  max_yaw_step_deg: 12.0
  dead_band: 0.002
  lost_tolerance: 2           # 连续丢帧数,超过则后退重找
  retreat_step: 0.08
  max_retreats: 3
  approach_speed: 0.08        # 修正段线速度

apriltag:                     # apriltag_ros 实际配置
  family: 36h11
  tag_size: 0.08
  detector.decimate: 1.0      # 必须 1.0:80 px 的 tag 经不起降采样
  tag_settle: 1.0             # 停稳后观测前的沉降等待(isaac 后端)

arm:
  free_space: ompl(RRTConnect)  approach: pilz_lin @ 0.15 倍速
  ik_attempts: 25  同分支约束: 直线可达才接受(6 分支尝试)

nav2:
  goal tolerance: 0.25 m / 0.25 rad → 里程计补位 → tag 闭环
  controller: MPPI(vx 0.5, wz 1.9)  planner: NavFn
  inflation_radius: 0.40  cost_scaling: 5.0  costmap: 纯静态层(仿真)

machine:                      # 与初版建议一致,已实现
  door: 10 s   clamp: 5 s   material: 3 s   machining: 8 s + 余量
```

> 仿真档参数;真机的速度、安全限值须按现场安全规范重新确认。

---

# 最终总结（已验证的落地组合）

```text
slam_toolbox            建图(绕场一圈,地图入包)
Nav2(NavFn+MPPI)       底盘粗导航(至 ~0.25 m)
里程计补位              最后一段直驱到示教停靠点(实施新增的一环)
AprilTag(apriltag_ros) 工位精停靠视觉闭环(实测 0.5–6 mm / ≤0.28°)
手眼校正帧 + TF2        坐标统一(标定误差可注入可度量)
头部 RGB + tag 平面      物料定位(观测误差实测 4–7 mm)
MoveIt 2(moveit_py)    IK、碰撞、大范围规划(同分支约束)
Pilz LIN                最后直线接近/退出(0.15 倍速,失败回退自由空间)
机器桥(MachineIO)      雕刻机通信与双侧互锁(仿真/Modbus 可换后端)
YAML 状态机             任务流程、超时、重试、恢复路由(数据即流程)
运动学底盘驱动           cmd_vel/odom(真机换底盘驱动,接口不变)
Isaac Sim               传感器级仿真(真渲染/真雷达,控制栈与真机同构)
```

实际走通的实施顺序（供真机阶段参考）：

```text
1. 运动学几何级闭环(RViz):先让流程和坐标系成立
2. 接口化:互锁桥、状态机、技能 Action(可复用的骨架)
3. 感知落地:真渲染+真解码,标定链自洽(此处暴露的问题最多)
4. 建图+导航:cmd_vel 底盘、slam、Nav2、补位衔接
5. 全链路回归与录制;异常路径逐个补齐
6. → 真机:硬件桥、标定四件、AMCL、安全设备(§18 待办)
```

一句话概括（在仿真中得到完整验证）：

> **Nav2 把机器人送到附近，里程计补完最后一米，AprilTag 把它修到毫米级，MoveIt 和直线插补完成安全上下料，夹具清隙消化最后几毫米,机器桥保证任何一步不在错误条件下发生——机器人从墙角出发，也回到墙角待命。**
