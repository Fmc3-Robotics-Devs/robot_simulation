# 技术参考登记

| 来源 | 版本/提交 | 本项目采用范围 |
|---|---|---|
| [NVIDIA Isaac Sim Python 安装](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_python.html) | 5.1.0 | Python 3.11、pip/uv 包版本、NVIDIA 索引和启动验证 |
| [NVIDIA Isaac Sim ROS 2 文档](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/index.html) | 5.1.0 | ROS 2 Bridge、相机、时钟、TF 与 Action Graph |
| [NVIDIA Isaac Sim USD 资产](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_overview.html) | 5.1.0 | 官方资产布局、Warehouse 与项目 USD 引用方式 |
| [NVIDIA Isaac Lab 文档](https://isaac-sim.github.io/IsaacLab/v2.3.2/) | 2.3.2 | external-project、任务配置和启动组织 |
| [NVIDIA Isaac Lab 项目结构](https://isaac-sim.github.io/IsaacLab/v2.3.2/source/overview/own-project/project_structure.html) | 2.3.2 | `source/<extension>/config/extension.toml` 与 Python module 声明 |
| [Unitree `unitree_sim_isaaclab` README](https://github.com/unitreerobotics/unitree_sim_isaaclab/blob/main/README.md) | 使用时记录 commit | 场景、观测、终止和任务注册的组织方式；不整体复制 |

实现新的第三方依赖或复制代码前，需补充具体链接、许可证和来源版本。用户提供的 [NVIDIA 6.0.1 AprilTag 资产页](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/assets/usd_assets_props.html#april-tags) 用于定位新版官方标签资产；若 5.1 本地资产中存在对应 USD，则优先使用本地 5.1 版本，不能直接把 6.x Python API 混入 5.1 运行代码。
