# Franzi 雕刻机上下料系统(仿真完整实现)

按《移动机械臂雕刻机自动上下料系统详细方案》在仿真中实现的完整 ROS 2 系统,
不含 VLA。四个包组成一条从任务到执行的链:

```text
franzi_task_manager        任务层:数据驱动状态机(states.yaml,方案 §13)
        │  8 个 ROS 2 Action
franzi_skills              技能层:导航/精停靠/检测/抓取/上下料/放置/安全姿态(§3)
        │  MoveItPy · TF2 · AprilTag · 底盘
franzi_pick_place          构件库:MobileBase、ArmMotion、Gripper、场景、tag 管线
        │
franzi_machine_bridge      雕刻机桥:互锁(§12.4/12.5)、超时(§12.6)、模拟 PLC
franzi_engraving_interfaces  msg / srv / action 定义
```

任务层只认得错误码和状态表;技能层只认得位姿和互锁;桥只认得信号。
每一层在真机上被替换时,其余层不动 —— 这是整个仓库的迁移策略。

## 构建

```bash
cd Rviz
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  franzi_engraving_interfaces franzi_machine_bridge \
  franzi_task_manager franzi_skills franzi_pick_place \
  franzi_description franzi_moveit_config
```

注意:若 `~/.local/bin` 里有 uv 安装的 Python,构建接口包前先从 PATH 去掉,
否则 rosidl 会拿错解释器(`ModuleNotFoundError: em`)。

## 运行完整周期

### RViz 后端(几何仿真,mock 检测)

```bash
ros2 launch franzi_skills tending_cell.launch.py
```

任务管理器延迟 40 s 启动(等 MoveItPy 就绪),随后自动跑完
IDLE → … → DONE 的完整雕刻周期。有用的参数:

- `rviz:=false` 无界面运行;
- `cycles:=2` 连续跑两个周期(成品自动放入下一个空格位);
- `autostart:=false` 只起服务,技能用 `ros2 action send_goal` 手动调;
- `task_delay:=60.0` 慢机器上加大启动延迟。

### Isaac 后端(真渲染图像 + 真 AprilTag 检测)

先起 Isaac(镜像模式,跟随 ROS 栈的关节与底盘位姿并发布相机/雷达):

```bash
IssacSim/run_ros2_cell.sh
```

再起 ROS 栈,感知切换为 apriltag_ros:

```bash
ros2 launch franzi_skills tending_cell.launch.py backend:=isaac
```

`backend:=isaac` 做三件事:关掉 mock 检测、启动 apriltag_ros(解码
`/head_d435/color/image_raw`,发布与 mock 同名的 `tag_N` TF 帧)、发布
`head_d435_Link → head_d435_optical` 的恒等静态变换(该 URDF 相机 link
本身就是光学朝向)。控制栈一字不改。

实测(2026-07,单周期):三站停靠误差 ≤ 3.2 mm / 0.11°(指标 8 mm/0.8°),
tag 检测的部件位姿与勘测差 7–8 mm,全周期 DONE。

录像(机载相机视角,真机上对 RealSense 同名 topic 同样可用;Isaac 需
`--cameras head_d435 left_wrist_d405` 启动才有腕部流):

```bash
ros2 run franzi_skills camera_recorder /head_d435/color/image_raw head.mp4 --fps 15
ros2 run franzi_skills camera_recorder /left_wrist_d405/color/image_raw wrist.mp4 --fps 15
```

成片样例在 `IssacSim/renders/`:`cell_overview.mp4`(上帝视角全景)、
`robot_cameras.mp4`(机载四相机 2×2 拼接)、`rviz_navigation.mp4`
(地图 + 规划路径 + 实走轨迹)。原始单路素材为同目录 `nav_*.mp4`。

OpenCV 写出的是 mp4v 编码,不少播放器不认;分发前转 H.264:

```bash
ffmpeg -i in.mp4 -c:v libx264 -pix_fmt yuv420p -movflags +faststart out.mp4
```

RViz 导航视图无头录屏:`Xvfb :77` + `rviz2 -d config/nav_view.rviz`
(`LIBGL_ALWAYS_SOFTWARE=1`)+ `ffmpeg -f x11grab -i :77`。

## 测试

```bash
python3 -m pytest src/franzi_machine_bridge/test src/franzi_task_manager/test src/franzi_skills/test
```

纯逻辑模块可脱离 ROS 测试:互锁(`interlocks.py`)、状态表
(`state_table.py`)、精停靠闭环(`dock_controller.py`)、格位簿(`slots.py`)。

## 建图与 Nav2 导航(方案 §8.2 + §18 阶段3/4)

真机同款链路:`cmd_vel` 底盘 + `/odom` + `/scan`(Isaac RTX 雷达)→
slam_toolbox 建图 → Nav2 粗导航 → AprilTag 精停靠。

```bash
# 建图(Isaac 跑着时):rsp + slam_toolbox + 绕场一圈,然后存图
ros2 launch franzi_moveit_config rsp.launch.py &
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=src/franzi_skills/config/slam.yaml &
ros2 run franzi_skills mapping_drive          # 自动巡场,结束后保持发布
ros2 run nav2_map_server map_saver_cli -f src/franzi_skills/maps/cell
# 存完 Ctrl-C 上面三个,重新 colcon build 让新图进 share

# 带导航跑整周期
ros2 launch franzi_skills tending_cell.launch.py backend:=isaac nav:=true
```

`nav:=true` 起 Nav2 四件套(NavFn 全局规划 + MPPI 控制 + behaviors +
bt_navigator,controller 直出 /cmd_vel)+ map_server。仿真中定位是恒等
`map→odom` 静态变换(无漂移的运动学 odom 即真值);真机换 AMCL /
slam_toolbox localization,taught 位姿与地图不变。velocity_smoother 与
collision_monitor 在真机 bringup 再启用(见 launch 内注释)。

导航目标 = 示教停靠位(odom)经当前 `map→odom` 换算后发出;到位后技能先按
里程计补完最后一段(粗导航容差 ~0.25 m,tag 在那个距离上解不出码),再进
tag 闭环把误差压到毫米级。

状态机含回家环节:启动于 `home` 待命点,上料后回 home 等加工,加工完回
机器取件,收尾回 home(dock_poses.yaml 里的 `home` 条目,无 tag)。

## 精停靠(方案 §8)

`precise_dock` 技能按 SEARCH → ALIGN → FINE → SETTLE → DOCKED 闭环:

1. 观测站位 tag(TagObserver,经手眼校正帧);
2. `dock_in_tag`(勘测:名义 tag 位姿 ⊖ 示教停靠位姿)把观测变成停靠目标;
3. 误差表达在停靠坐标系(§8.3,最短角差);
4. 限步长、带死区地逼近,连续 N 帧进容差才算 DOCKED(§8.7);
5. tag 丢失 → 等待 → 后退重找 → 有限次后报 `TAG_LOST`(§14.1)。

决策全部在 `dock_controller.py`(纯函数,已测);执行器是注入的 —— 仿真里是
运动学底盘的小步 `drive_to(exact=True)`,真机上换成 cmd_vel 速度环即可。

## robot_clear 的编排

`robot/signals` 只有任务管理器一个写者(全量消息,增量语义)。状态表用
`signals_on_entry` / `signals_on_success` 声明:进入 LOAD/UNLOAD 前拉低
`robot_clear`,EXIT_MACHINE 成功后才拉高;FAULT 态刻意不声明 clear ——
故障的手臂可能停在机器里,说它清空是door 关到手臂上的第一步。

## 迁移到真机:逐层接缝

| 仿真组件 | 真机替换 | 其余各层 |
|---|---|---|
| `SimulatedMachine`(machine_io.py) | 实现 `MachineIO` 的 Modbus/IO 类 | 桥、互锁、任务层不动 |
| `machine/sim/material_present` 话题 | 删掉:机器自己的传感器经 MachineIO 上报 | — |
| `MockTagDetector` | apriltag_ros(Isaac 后端已经在用) | TagObserver、技能不动 |
| `MobileBase.drive_to`(导航) | Nav2 `NavigateToPose`(skill_client 已按此形状建模) | 状态表不动 |
| `MobileBase.drive_to(exact)`(停靠步) | cmd_vel 速度环(增益/限幅在 dock_controller) | 决策逻辑不动 |
| demo 虚拟控制器 | `franzi_hardware_bridge` + franzi_sdk(见 Rviz/NextStep.md) | MoveIt、技能不动 |
| Isaac 相机 | RealSense 驱动(topic 布局已按其命名) | apriltag、技能不动 |
| 手眼:`handeye_correction` 参数 | 真标定结果填入同一参数 | 校正帧机制不动 |
| `dock_poses.yaml`(示教停靠) | 真机开到位重新示教(teach_docks) | — |

硬性提醒(方案 §12.7):软件互锁不替代急停、安全继电器、门开关。
`interlocks.py` 存在的意义是不让机器人**请求**不可能的事,不是让不安全的
机器变安全。

## 尚未覆盖

- 接触物理:抓取仍是运动学 attach(Isaac 目前是镜像渲染,不做接触);
- Nav2/SLAM 集成(雷达已在 Isaac 中发布 `/scan`、`/mid360/points`,可以接);
- 多物料、随机来料(方案 §21 属 VLA 阶段,按目标排除)。
