"""The whole tending cell: MoveIt backend, machine bridge, skills, task.

Four things come up, in dependency order:

  1. the MoveIt demo backend - move_group, robot_state_publisher and the
     virtual trajectory controllers (replaced by Isaac or hardware drivers
     later; nothing above them knows which is running);
  2. the machine bridge, simulating the engraving machine's PLC;
  3. the skill server, which owns the arm, the base and the planning scene;
  4. the task manager, delayed until the skills have finished bringing up
     MoveItPy, which then runs the cycle and exits.

The cell layout is read from franzi_pick_place/config/task.yaml and handed to
every node that needs it, so there is exactly one description of the cell.

`backend:=isaac` swaps perception, not control: the mock tag detector goes
away and apriltag_ros decodes the head camera images that Isaac renders
(start `IssacSim/run_ros2_cell.sh` alongside). The motion stack is unchanged -
Isaac mirrors `/joint_states` and `/base/pose` and exists to be looked at,
which is exactly the seam a camera driver plus real robot will later fill.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def cell_parameters():
    """task.yaml flattened, so any node can take it regardless of its name."""
    task_yaml = (
        Path(get_package_share_directory("franzi_pick_place")) / "config" / "task.yaml"
    )
    return yaml.safe_load(task_yaml.read_text())["pick_place_task"]["ros__parameters"]


def generate_launch_description():
    moveit_launch = Path(get_package_share_directory("franzi_moveit_config")) / "launch"
    pick_place_share = Path(get_package_share_directory("franzi_pick_place"))
    skills_share = Path(get_package_share_directory("franzi_skills"))

    cell = cell_parameters()
    skills_yaml = skills_share / "config" / "skills.yaml"
    skills = yaml.safe_load(skills_yaml.read_text())["skill_server"]["ros__parameters"]

    # Real perception for the Isaac backend: apriltag_ros decodes the head
    # camera stream into the same tag_N TF frames the mock publishes, so the
    # TagObserver cannot tell the difference - which is the point.
    tag_ids = [cell["tag"][f"{station}_id"] for station in ("feeder", "machine", "outfeed")]
    apriltag_parameters = {
        "family": "36h11",
        "size": float(cell["tag"]["size"]),
        "profile": False,
        "max_hamming": 0,
        "detector.threads": 2,
        # The head camera sees an 80 px tag at working distance; decimation
        # would halve that below what the decoder needs.
        "detector.decimate": 1.0,
        "detector.blur": 0.0,
        "detector.refine": True,
        "detector.sharpening": 0.25,
        "pose_estimation_method": "pnp",
        "tag.ids": tag_ids,
        "tag.frames": [f"tag_{tag_id}" for tag_id in tag_ids],
        "tag.sizes": [float(cell["tag"]["size"])] * len(tag_ids),
    }
    isaac_perception = [
        # The camera link points z out of the lens but its x axis is not the
        # image row direction: the imager sits a quarter turn rolled, so the
        # optical frame is link * Rz(-90 deg). Isaac prints the same transform
        # at start-up as a cross-check.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="head_optical_tf",
            arguments=[
                "--yaw", "-1.5707963",
                "--frame-id", "head_d435_Link",
                "--child-frame-id", "head_d435_optical",
            ],
            condition=LaunchConfigurationEquals("backend", "isaac"),
        ),
        # apriltag pairs image and camera_info by exact stamp; Isaac's bridge
        # stamps them from two separate graph nodes, so the relay re-stamps
        # the (static) intrinsics with each image's own time. A real camera
        # driver stamps both together and needs neither the relay nor the
        # remap below.
        Node(
            package="franzi_skills",
            executable="camera_stamp_relay",
            name="camera_stamp_relay",
            output="log",
            parameters=[{"camera": "head_d435"}],
            condition=LaunchConfigurationEquals("backend", "isaac"),
        ),
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            output="log",
            remappings=[
                ("image_rect", "/head_d435/color/image_raw"),
                ("camera_info", "/head_d435/color/synced_camera_info"),
            ],
            parameters=[apriltag_parameters],
            condition=LaunchConfigurationEquals("backend", "isaac"),
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "backend",
                default_value="rviz",
                description="rviz: mock tag detections. isaac: apriltag_ros on "
                "the Isaac camera stream (run IssacSim/run_ros2_cell.sh too).",
            ),
            DeclareLaunchArgument(
                "localization",
                default_value="amcl",
                description="amcl: lidar localization against the map, the "
                "hardware pipeline. static: identity map->odom for debugging.",
            ),
            DeclareLaunchArgument(
                "scene",
                default_value="warehouse",
                description="Which world the cell stands in: the original "
                "warehouse, or the 智能制造中心 shopfloor (start Isaac with "
                "--scene center to match). Selects the map and the taught "
                "standby pose; the three station docks are identical.",
            ),
            DeclareLaunchArgument(
                "nav",
                default_value="false",
                description="Coarse navigation through Nav2 on the built map "
                "(map_server + AMCL + planner/controller) instead of the "
                "kinematic base's direct drive.",
            ),
            # Localization exactly as the real cell runs it: AMCL closes
            # map -> odom from the lidar against the built map. The kinematic
            # odometry is simply a very good odom to it. `localization:=static`
            # falls back to the identity transform for debugging runs where
            # the particle filter itself is under suspicion.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_odom_tf",
                arguments=["--frame-id", "map", "--child-frame-id", "odom"],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("nav"),
                            "' == 'true' and '",
                            LaunchConfiguration("localization"),
                            "' == 'static'",
                        ]
                    )
                ),
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="log",
                parameters=[
                    str(skills_share / "config" / "nav2.yaml"),
                    {
                        # The taught standby pose of the active scene: where
                        # skill_server wakes the base up.
                        "initial_pose.x": ParameterValue(
                            PythonExpression(
                                ["0.0 if '", LaunchConfiguration("scene"), "' == 'center' else -7.0"]
                            ),
                            value_type=float,
                        ),
                        "initial_pose.y": ParameterValue(
                            PythonExpression(
                                ["-6.8 if '", LaunchConfiguration("scene"), "' == 'center' else -7.0"]
                            ),
                            value_type=float,
                        ),
                        "initial_pose.yaw": 0.67,
                    },
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("nav"),
                            "' == 'true' and '",
                            LaunchConfiguration("localization"),
                            "' == 'amcl'",
                        ]
                    )
                ),
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="log",
                parameters=[
                    {
                        "yaml_filename": PythonExpression(
                            [
                                "'",
                                str(skills_share / "maps"),
                                "/center.yaml' if '",
                                LaunchConfiguration("scene"),
                                "' == 'center' else '",
                                str(skills_share / "maps"),
                                "/cell.yaml'",
                            ]
                        )
                    }
                ],
                condition=IfCondition(LaunchConfiguration("nav")),
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="log",
                parameters=[{"autostart": True, "node_names": ["map_server"]}],
                condition=IfCondition(LaunchConfiguration("nav")),
            ),
            # Nav2, reduced to the four servers this cell needs: global
            # planner, controller, behaviors and the BT navigator. The
            # controller publishes /cmd_vel directly - no velocity smoother,
            # no collision monitor. Both are worth their complexity on
            # hardware; here the smoother's configure step refuses to come up
            # and the monitor needs the very scan timing this stage exists to
            # sidestep. The full bringup is one include away when they earn
            # their place back.
            *[
                Node(
                    package=package,
                    executable=executable,
                    name=name,
                    output="log",
                    parameters=[str(skills_share / "config" / "nav2.yaml")],
                    condition=IfCondition(LaunchConfiguration("nav")),
                )
                for package, executable, name in [
                    ("nav2_planner", "planner_server", "planner_server"),
                    ("nav2_controller", "controller_server", "controller_server"),
                    ("nav2_behaviors", "behavior_server", "behavior_server"),
                    ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
                ]
            ],
            # Delayed behind the map manager: the global costmap's static
            # layer blocks its activation on /map, and on this machine that
            # wait has been seen to outlast the manager's service timeout,
            # aborting the whole bringup. Letting the map go active first
            # removes the race; the skill layer's lifecycle kick (on
            # NAV_REJECTED) remains as the second line of defence.
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",
                        output="log",
                        parameters=[
                            {
                                "autostart": True,
                                "bond_timeout": 20.0,
                                "service_introspection_mode": "disabled",
                                "node_names": [
                                    "planner_server",
                                    "controller_server",
                                    "behavior_server",
                                    "bt_navigator",
                                ],
                            }
                        ],
                        condition=IfCondition(LaunchConfiguration("nav")),
                    )
                ],
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Run the engraving cycle as soon as everything is up.",
            ),
            DeclareLaunchArgument(
                "cycles", default_value="1", description="How many cycles to run."
            ),
            DeclareLaunchArgument(
                "task_delay",
                default_value="40.0",
                description="Seconds to wait for the skill server before the task starts.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(moveit_launch / "demo.launch.py")),
                launch_arguments={
                    "use_rviz": "false",
                    # The skill server's base driver owns the chassis pose and
                    # the wheel joints.
                    "publish_virtual_joint_tf": "false",
                    "publish_chassis_joints": "false",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", str(pick_place_share / "config" / "pick_place.rviz")],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
            Node(
                package="franzi_machine_bridge",
                executable="machine_bridge",
                name="machine_bridge",
                output="screen",
            ),
            # A few seconds behind move_group, so MoveItPy finds the planning
            # scene services on its first try instead of its tenth.
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="franzi_skills",
                        executable="skill_server",
                        name="skill_server",
                        output="screen",
                        parameters=[
                            cell,
                            skills,
                            {
                                "use_nav2": ParameterValue(
                                    PythonExpression(
                                        ["'", LaunchConfiguration("nav"), "' == 'true'"]
                                    ),
                                    value_type=bool,
                                ),
                                "mock_tags.enabled": ParameterValue(
                                    PythonExpression(
                                        ["'", LaunchConfiguration("backend"), "' != 'isaac'"]
                                    ),
                                    value_type=bool,
                                ),
                                # A real detection pipeline has latency the
                                # mock does not: wait out a full frame or two.
                                "tag_settle": ParameterValue(
                                    PythonExpression(
                                        [
                                            "1.0 if '",
                                            LaunchConfiguration("backend"),
                                            "' == 'isaac' else 0.3",
                                        ]
                                    ),
                                    value_type=float,
                                ),
                                "dock.observe_timeout": ParameterValue(
                                    PythonExpression(
                                        [
                                            "3.0 if '",
                                            LaunchConfiguration("backend"),
                                            "' == 'isaac' else 1.5",
                                        ]
                                    ),
                                    value_type=float,
                                ),
                                # Loose material is measured in the depth
                                # image when there are real images to measure
                                # in; the RViz backend has none and infers
                                # from the tag as before.
                                "material_detection.mode": ParameterValue(
                                    PythonExpression(
                                        [
                                            "'vision' if '",
                                            LaunchConfiguration("backend"),
                                            "' == 'isaac' else 'tag'",
                                        ]
                                    ),
                                    value_type=str,
                                ),
                                # The shopfloor scene keeps its own standby
                                # pose; empty falls back to the warehouse book.
                                "dock_poses_file": ParameterValue(
                                    PythonExpression(
                                        [
                                            "'",
                                            str(
                                                pick_place_share
                                                / "config"
                                                / "dock_poses_center.yaml"
                                            ),
                                            "' if '",
                                            LaunchConfiguration("scene"),
                                            "' == 'center' else ''",
                                        ]
                                    ),
                                    value_type=str,
                                ),
                            },
                        ],
                    )
                ],
            ),
            *isaac_perception,
            TimerAction(
                period=LaunchConfiguration("task_delay"),
                actions=[
                    Node(
                        package="franzi_task_manager",
                        executable="task_manager",
                        name="task_manager",
                        output="screen",
                        parameters=[
                            {
                                "autostart": ParameterValue(
                                    LaunchConfiguration("autostart"), value_type=bool
                                ),
                                "cycles": ParameterValue(
                                    LaunchConfiguration("cycles"), value_type=int
                                ),
                            }
                        ],
                    )
                ],
            ),
        ]
    )
