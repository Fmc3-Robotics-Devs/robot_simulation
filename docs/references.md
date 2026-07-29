# 技术参考登记

| 来源 | 版本/提交 | 本项目采用范围 |
|---|---|---|
| [NVIDIA Isaac Sim Python 安装](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_python.html) | 5.1.0 | Python 3.11、pip/uv 包版本、NVIDIA 索引和启动验证 |
| [NVIDIA Isaac Sim ROS 2 文档](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/index.html) | 5.1.0 | ROS 2 Bridge、相机、时钟、TF 与 Action Graph |
| [NVIDIA Isaac Sim USD 资产](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_overview.html) | 5.1.0 | 官方资产布局、Warehouse 与项目 USD 引用方式 |
| [NVIDIA Isaac Lab 文档](https://isaac-sim.github.io/IsaacLab/v2.3.2/) | 2.3.2 | external-project、任务配置和启动组织 |
| [NVIDIA Isaac Lab 项目结构](https://isaac-sim.github.io/IsaacLab/v2.3.2/source/overview/own-project/project_structure.html) | 2.3.2 | `source/<extension>/config/extension.toml` 与 Python module 声明 |
| [Hugging Face Hub 下载文档](https://huggingface.co/docs/huggingface_hub/en/package_reference/file_download) | 使用时记录 `huggingface_hub` 版本 | 固定 revision、最小文件下载、缓存和 dry-run |
| [Hugging Face Pickle 安全说明](https://huggingface.co/docs/hub/security-pickle) | 在线文档 | 外部权重/数据按不可信输入处理；平台扫描不能替代本地审查 |
| [Unitree `unitree_sim_isaaclab` README](https://github.com/unitreerobotics/unitree_sim_isaaclab/blob/main/README.md) | 使用时记录 commit | 场景、观测、终止和任务注册的组织方式；不整体复制 |

实现新的第三方依赖或复制代码前，需补充具体链接、许可证和来源版本。用户提供的 [NVIDIA 6.0.1 AprilTag 资产页](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/assets/usd_assets_props.html#april-tags) 用于定位新版官方标签资产；若 5.1 本地资产中存在对应 USD，则优先使用本地 5.1 版本，不能直接把 6.x Python API 混入 5.1 运行代码。

## Hugging Face 数据参考

Hugging Face 作为项目的第三层资料源：优先查 NVIDIA、机器人厂商或许可证清楚的仓库，并固定到完整 commit SHA。下面的登记表示“允许评估”，不表示已将数据、模型或 USD 纳入正式场景。

| 来源 | 固定版本与许可证 | 当前采用范围 | 使用限制 |
|---|---|---|---|
| [NVIDIA G1 Locomanipulation Dataset v1](https://huggingface.co/datasets/nvidia/g1_locomanip_dataset) | `558da5ba581dfd4bf02a459c70b208f2ac5d1774`；CC-BY-4.0 | 优先参考“抓取—导航—放置”的阶段划分、状态/动作字段、成功样本导出和 LeRobot/HDF5 组织 | G1 与 Wheel Bot 的关节、底盘和相机不同；不得直接回放其动作；数据卡明确其为 SDG 示例，不用于生产部署 |
| [NVIDIA PhysicalAI SimReady Warehouse 01](https://huggingface.co/datasets/nvidia/PhysicalAI-SimReady-Warehouse-01) | `c7fe115cb79c7ddbd0532630d7768b5736b0ecc4`；CC-BY-4.0 | 候选仓储 OpenUSD、货架和道具；用于扩充工位语义和对比现有 Warehouse | 数据卡面向 Isaac Sim 4.x；进入 5.1 前必须通过依赖、单位、材质、Collider 与 GUI/物理复验；不覆盖项目物理层 |
| [NVIDIA GR00T X Embodiment Sim](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-GR00T-X-Embodiment-Sim) | `ea7ac0b68f87da62f1e726771bba0fe74300802f`；CC-BY-4.0 | 只评估 `Transport`、`BoxCleanup`、`PlateToCardboardBox` 等相关 subset 的任务分段和双臂协作 schema | 全库规模极大且混合多种机器人；禁止整库下载或直接回放 |
| [NVIDIA PhysicalAI Robotics Manipulation Augmented](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-Manipulation-Augmented) | `9a9167bcf5e59f130b115395c5da411c11e3cebc`；README 声明 CC-BY-4.0，Hub license 字段为空 | 只读数据卡和 HDF5 schema，参考 Isaac Lab 示范、腕部 RGB/深度等多模态记录流程 | 仓库含 Python 和 `.pth`；禁止执行脚本或下载/加载权重。Franka 单臂堆叠任务不作为 Wheel Bot 控制数据 |
| [NVIDIA PhysicalAI Robotics Manipulation Objects](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-Manipulation-Objects) | `538b6833148b4731e4ea30fe181cc4eecfc13440`；NVIDIA OneWay Noncommercial | 仅参考 Kinova 双臂自动抓放的数据字段、规划和成功判定思路 | 非商业条款；不复制进交付、不用于商业训练或派生资产 |
| [NVIDIA Synthetic Warehouse Operations Scenes](https://huggingface.co/datasets/nvidia/PhysicalAI-WorldModel-Synthetic-Warehouse-Operations-Scenes) | `d5b88d3abcf659f304a107f4336b71b4e2159133`；自定义 OpenMDW-1.1 | 只读 warehouse box-pickup 的多相机、灯光/场景随机化、随机种子和 WebDataset 元数据说明 | 条款复核前不下载数据或生成派生物；数据规模很大且不是可执行抓箱轨迹 |
| [社区：OpenArm Pick v6](https://huggingface.co/datasets/AiSaurabhPatil/openarm_pick_v6) | `67bf78fd876ed51c57d5c41bff01f59e50274441`；Apache-2.0 | 参考 Isaac Sim 双臂、头部与双腕相机的 LeRobot v2.1 命名、时间序列和遥操作记录结构 | 未验证作者来源；OpenArm 的 16 维动作与 Wheel Bot 不同，不直接映射关节 |
| [社区：Franka Place Cube in Box Mimic](https://huggingface.co/datasets/Ekshan267/franka-place-cube-in-box-mimic-dataset) | `7d01ed3c84eb723da88351d092bc2fac88993560`；Apache-2.0 | 参考 Isaac Lab Mimic 标注、腕部/顶视相机和“放入箱体”成功条件 | 未验证作者来源；单臂方块任务，仅迁移数据结构和测试思路 |
| [Unitree G1 Dex3 Object Placement](https://huggingface.co/datasets/unitreerobotics/G1_Dex3_ObjectPlacement_Dataset) | `2e1d63e5fd9587e1240b889c51c6553be6910027`；Apache-2.0 | 参考双臂将物体放入蓝色料箱的任务分段、28 维状态/动作以及头部双目和腕部画面组织 | 数据卡未声明由 Isaac Sim 生成；只迁移任务和数据 schema，不当作仿真轨迹 |
| [Unitree G1 Dex3 Camera Packaging](https://huggingface.co/datasets/unitreerobotics/G1_Dex3_CameraPackaging_Dataset) | `c52f7483347c1901739c025a640982a22a44a70d`；Apache-2.0 | 后续手机场景参考：双臂把 D405 放入包装盒并合盖的动作阶段、多相机和 LeRobot v2.0 结构 | 物体、手型和盒体不同；不得替代手机/泡沫箱 STL 与本项目插槽规则 |
| [社区：GR1 Manipulation Isaac Sim Rendered](https://huggingface.co/datasets/khang123452/GR1-Manipulation-IsaacSim-Rendered) | `77a04466eb4cadc3061056c68f8f33e842df7a81`；CC-BY-4.0 | 视觉参考：RGB、Depth、Segmentation 同步渲染和箱体类 pick-and-place 镜头 | 未验证作者来源；数据由 MuJoCo 轨迹转 USD 渲染，不是 Wheel Bot 的 Isaac Lab 可执行轨迹 |
| [社区：MAS-VLN Randomized Warehouse RGBD](https://huggingface.co/datasets/yang-jiao/mas-vln-isaac-rgbd) | `e4c4ea52c8149cfe1a80b2d7b82d2d852faa06ba`；未声明许可证 | 只读 Isaac Sim 5.1 仓储 RGBD、Parquet 索引和可复现 seed 的公开数据卡 | 未验证作者来源；在许可证补齐前禁止下载、复制、训练或生成交付物 |

采用顺序为：先读数据卡、README 和许可证原文，再下载最小元数据/预览，再做离线兼容测试，最后才决定是否形成项目派生资产。任何采用都要记录 Hub commit、许可证原文、署名与修改说明、再分发许可、下载文件清单和 SHA-256；禁止使用未固定的 `main` 作为可复现实验输入。Python API 使用 `allow_patterns`，`hf download` CLI 使用 `--include`。

`unitreerobotics/G1_Dex1_PackPhone` 虽与手机任务同名，但在固定版本 `db5df4f18af37fb09e600b0abe93ef3ce283f2ed` 中没有许可证和可用数据卡，因此当前明确拒绝下载或采用。
