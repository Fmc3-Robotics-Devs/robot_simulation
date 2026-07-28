"""Bring up the MoveIt demo plus the machine-tending task.

The MoveIt demo supplies move_group, the robot state publisher and the virtual
trajectory controllers; this file adds the RViz config that also shows the
station status light, and the task node itself.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_launch = Path(get_package_share_directory("franzi_moveit_config")) / "launch"
    task_share = Path(get_package_share_directory("franzi_pick_place"))

    # RViz is started here rather than through the MoveIt config's
    # moveit_rviz.launch.py: that file shares the `use_rviz` argument name with
    # demo.launch.py, and including both silently swallowed the RViz node.
    moveit_config = (
        MoveItConfigsBuilder("wheel_robot_4.0", package_name="franzi_moveit_config")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
        )
        .pilz_cartesian_limits()
        .to_moveit_configs()
    )

    show_rviz = LaunchConfiguration("rviz")
    task_config = LaunchConfiguration("task_config")
    start_delay = LaunchConfiguration("start_delay")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(task_share / "config" / "pick_place.rviz")],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(show_rviz),
    )

    task_node = Node(
        package="franzi_pick_place",
        executable="pick_place_task",
        name="pick_place_task",
        output="screen",
        # The override has to come after the file for it to win.
        parameters=[
            task_config,
            {"repeat": ParameterValue(LaunchConfiguration("repeat"), value_type=int)},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "task_config",
                default_value=str(task_share / "config" / "task.yaml"),
                description="Cell layout and task parameters.",
            ),
            DeclareLaunchArgument(
                "start_delay",
                default_value="6.0",
                description="Seconds to wait for move_group before the task starts.",
            ),
            DeclareLaunchArgument(
                "repeat",
                default_value="1",
                description="How many load/machine/unload cycles to run.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(moveit_launch / "demo.launch.py")),
                launch_arguments={
                    "use_rviz": "false",
                    # The task node's base driver owns the chassis pose and the
                    # wheel joints; the demo defaults would pin the robot at the
                    # origin and fight it.
                    "publish_virtual_joint_tf": "false",
                    "publish_chassis_joints": "false",
                }.items(),
            ),
            rviz_node,
            TimerAction(period=start_delay, actions=[task_node]),
        ]
    )
