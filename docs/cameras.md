# 四路相机

Wheel Bot 的原始 `wheel_robot_4.0.urdf`（`origin/franzi`）已经包含 D435/D405 的实体 link 和固定 joint；本工程不复制或第二次建模这些壳体。相机传感器配置集中在 `franzi_sim.cameras`，由 `usd/assets/robots/wheel_bot/camera_sensors.usda` 作为传感器层交付；可直接引用的完整机器人入口是 `usd/assets/robots/wheel_bot/wheel_bot_with_cameras.usda`。

| 逻辑名 | URDF 父 link | 相机 | ROS 图像话题 | ROS CameraInfo |
| --- | --- | --- | --- | --- |
| `head_d435` | `head_d435_Link` | D435，1280×720 | `/franzi/camera/head_d435/color/image_raw` | `/franzi/camera/head_d435/color/camera_info` |
| `chest_d435` | `body_d435_Link` | D435，1280×720 | `/franzi/camera/chest_d435/color/image_raw` | `/franzi/camera/chest_d435/color/camera_info` |
| `left_wrist_d405` | `left_wrist_d405_Link` | D405，1280×720 | `/franzi/camera/left_wrist_d405/color/image_raw` | `/franzi/camera/left_wrist_d405/color/camera_info` |
| `right_wrist_d405` | `right_wrist_d405_Link` | D405，1280×720 | `/franzi/camera/right_wrist_d405/color/image_raw` | `/franzi/camera/right_wrist_d405/color/camera_info` |

四路分辨率均为 1280×720，USD `clippingRange` 统一为 **`(0.05, 100) m`**。USD Camera 使用 `+Y` 向上、`-Z` 向前；ROS optical frame 使用 `+X` 向右、`+Y` 向下、`+Z` 向前。二者不能共用同一个 Xform，因此每个壳体下分别交付 `*_camera_mount` 和 `*_optical_frame` 两个 sibling：

| 相机 | USD Camera mount XYZ / RPY（度） | ROS optical XYZ / RPY（度） |
| --- | --- | --- |
| 头部 D435 | `(0,0,0)` / `(180,0,-90)` | `(0,0,0)` / `(0,0,-90)` |
| 胸部 D435 | `(0,0,0)` / `(180,0,-90)` | `(0,0,0)` / `(0,0,-90)` |
| 左腕 D405 | `(0,0,-0.0235)` / `(0,0,90)` | `(0,0,-0.0235)` / `(180,0,90)` |
| 右腕 D405 | `(0,0,-0.0235)` / `(0,0,90)` | `(0,0,-0.0235)` / `(180,0,90)` |

双腕 D405 STL 的两个圆形镜头位于 housing local `-Z` 面，网格范围为 `z=[-0.023,0] m`。项目交付 URDF 已把两条 D405 fixed joint 的 RPY 修正为 identity，使 housing `-Z` 与 wrist-roll/手指延伸方向 `-Z` 同轴；USD Camera 的 `-Z` 光轴再与壳体 `-Z` 对齐。因此夹爪自然下垂时，夹爪、实体镜头和相机观察方向都朝地。`Rz=90°` 只校正画面横竖方向，使左/右画面分别把机器人放在外侧边缘，不改变光轴。当前单一 RGB Camera 建模为两镜头之间 `x=y=0` 的**虚拟双目中点**，位于镜头面外 0.5 mm，并不冒充某一个真实 RGB 镜头的已标定光心。

头部和胸部 D435 直接随原 URDF 的 `head_d435_joint` / `body_d435_joint` 固定位置运动；左右腕 D405 分别随同名 wrist fixed joint 运动，绝不交叉映射。

## 静态检查

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest tests/test_camera_configuration.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest tests/test_optical_frame_tf.py tests/test_apriltag_assets.py
uv run python scripts/verify_camera_configuration.py
uv run python scripts/verify_optical_frame_tf.py
```

## Isaac GUI 画面复核

在正式场景 `usd/scenes/warehouse_box_transfer.usda` 中，Wheel Bot 位于 `/WheelBotBoxTransfer/WheelBot`。启动时间线后，逐一在 Camera 视口选择以下 prim：

`/WheelBotBoxTransfer/WheelBot/head_d435_Link/head_d435_camera_mount/camera`、`/WheelBotBoxTransfer/WheelBot/body_d435_Link/chest_d435_camera_mount/camera`、`/WheelBotBoxTransfer/WheelBot/left_wrist_d405_Link/left_wrist_d405_camera_mount/camera`、`/WheelBotBoxTransfer/WheelBot/right_wrist_d405_Link/right_wrist_d405_camera_mount/camera`。

每路保存一张 viewport 截图，并确认头/胸画面面向前方，左右腕画面与逻辑名称对应。最新正式工位 RTX 截图与总览归档到 [`evidence/workcell/2026-07-30/final/`](../evidence/workcell/2026-07-30/final/)；证据脚本使用双臂全零的自然下垂姿态，要求两侧 `camera forward == housing -Z == wrist-roll/夹爪 -Z == world down`，同时检查画面 roll，以及中心光轴前 0.3 m 内没有腕部或夹爪的首次碰撞。外侧画面边缘允许保留少量机器人本体结构，用于对应真实 D405 安装视角。中立腕相机不要求看见或解码任何 Tag。随箱移动的 Tag 0 和静态桌角 Tag 1/2 分别由独立近景或后续实际任务姿态验证，不绑定固定的左腕/右腕归属。该姿态只用于传感器取证，不代表 MoveIt 已规划出预抓轨迹。

当前 `Image` / `CameraInfo` 的 header 使用 `*_optical_frame`。`franzi_sim.optical_frame_tf` 为每个 URDF 相机壳体 link 发布独立的 ROS optical 固定边：头/胸 D435 为 `(0,0,-90)`，双腕 D405 为 `(180,0,90)`（XYZ Euler 度）。双腕变换由 USD mount `(0,0,90)` 再乘局部 `Rx(180°)` 得到；运动链其余部分仍由 URDF/robot-state publisher 发布。

在启用 ROS 2 bridge 后，以已有 `rclpy` node 调用：

```python
from franzi_sim.optical_frame_tf import publish_optical_frame_static_transforms

publish_optical_frame_static_transforms(node)
```

它使用 ROS 2 的 `tf2_ros.StaticTransformBroadcaster` 在 `/tf_static` 发布四条可持久化边。离线检查不能代替真实 DDS 验证：在 Isaac/ROS 环境中应运行 `ros2 run tf2_ros tf2_echo head_d435_Link head_d435_optical_frame`（其余三路同理），并确认 `/tf_static` 中每个 child frame 只有一个发布者。

启用 `isaacsim.ros2.bridge` 后，调用 `franzi_sim.ros2_camera_bridge.add_ros2_camera_publishers()` 会为每路相机创建独立的 Render Product、`ROS2CameraHelper` 和 `ROS2CameraInfoHelper`。因此图像与内参均来自同一个 Camera prim，不维护第二套手写内参。

坐标转换遵循 [NVIDIA Isaac Sim 5.1 Conventions](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/reference_material/reference_conventions.html)：从 USD camera 到 ROS camera 需在相机局部 X 轴旋转 180°。

## ROS 2 运行验证

在项目 Isaac Sim 已启动正式场景 ROS 2 Bridge 时，于系统 Jazzy 终端运行以下命令，以真实 DDS 消息生成验证证据：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 scripts/verify_ros2_camera_topics.py \
  --timeout 45 \
  --output evidence/cameras/YYYY-MM-DD/formal_ros2/ros2_topics.json
```

验证器不会分别接受每个 topic 的首包。每路 `Image` 和 `CameraInfo` 只有在 `header.stamp.sec` 与 `header.stamp.nanosec` 都相同后才会计入 `topics`，并在 `matched_pairs` 写入相机名、两个 topic 与该精确时间戳。`passed: true` 还要求四路都已配对、所有字段契约有效；没有匹配的端点会保留在 `missing_topics`，而已到达但未配对的端点另列在 `unmatched_topics`。该严格验证器已实现，但最新正式场景的 live ROS 2、TF 和 Tag 位姿闭环尚未重采；历史证据只能说明当时采集到的消息，不能因验证器改动而回填或宣称已经通过这一同步门。
