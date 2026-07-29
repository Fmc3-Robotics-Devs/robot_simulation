# franzi_sim

这是 Wheel Bot 的 Isaac Lab external-project 包。`scenarios/` 定义可在无 GPU、无 Isaac 环境下测试的任务契约；`cameras.py`、`camera_runtime.py` 与 `ros2_camera_bridge.py` 分别负责四相机唯一配置、Isaac Camera prim 和 ROS 2 Image/CameraInfo 发布图。后续 Isaac Lab 环境、ROS 2 Action 服务器和 MoveIt 执行器以适配器方式调用这些契约。

首个任务是 `warehouse_box_transfer`。它将标签搜索、底盘接近、双臂抓取、运输、放置与退出明确划分为状态，便于对每个执行器实施独立重试与日志记录。

当前已交付任务编排骨架、四相机运行适配与 Warehouse foundation；AprilTag、TF optical-frame、底盘和双臂执行器仍按 P2/P3 接入，不把 foundation 误报为搬箱闭环。

`config/extension.toml` 按 Isaac Lab 2.3.2 external-project 结构声明扩展；仓库根 `pyproject.toml` 是唯一的 uv 环境与安装入口，`source/franzi_sim/pyproject.toml` 只保留扩展自身的包元数据，不单独建立第二套虚拟环境。
